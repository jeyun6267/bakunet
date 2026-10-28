/* =========================================================================
   Brake Board (Robust & Anti-Hunting Version + Human Brake Protection)
   - ASMS : mode는 첫 바이트만 신뢰
   - PC   : 자율주행(mode == 1)일 때만 brake bit 신뢰
   - 수동 : mode != 1일 때 PC brake 신호를 항상 무시
   - 모터 : 불감대 + 히스테리시스 + 센서 필터 적용
   - 추가 : 브레이크/긴급정지 중 사람이 브레이크를 더 밟으면 액추에이터가 밀어내지 않음
   - 수정 : mode == 3인 동안 긴급정지 브레이크를 계속 유지
   - 수정 : Due 12-bit/3.3V ADC 전압 보정, 0.4801V ON / 0.4799V OFF
   - 수정 : D5 릴레이 Active HIGH (모듈에 따라 변경 가능)
   - PC brake: 70% 이동 -> 300ms 유지 -> 풀브레이킹 (시리얼 t 입력 없음)
   - ASMS mode 3: PC 단계/대기 무시, 즉시 풀브레이킹 목표 적용
   ========================================================================= */

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <math.h>

// ================= [센서/모터 핀 및 설정] =================
const int PIN_SENSOR = A1;
const int PIN_MT_FWD = 8;   // rpwm
const int PIN_MT_BWD = 7;   // lpwm

// ================= [브레이크등 설정] =================
// Ethernet Shield 사용 시 D4는 SD 카드 CS이므로 릴레이에 사용하지 않음.
const int PIN_BRAKE_PRESSURE = A0;
const int PIN_BRAKE_RELAY    = 5;

// 실제 릴레이 모듈이 LOW 입력에서 작동하면 true, HIGH에서 작동하면 false.
// 모듈 IN이 HIGH일 때 작동한다고 가정하여 false로 설정.
// 릴레이 접점의 COM-NO 연결 및 실제 작동을 별도로 확인해야 함.
const bool BRAKE_RELAY_ACTIVE_LOW = false;
const int BRAKE_RELAY_ON  = BRAKE_RELAY_ACTIVE_LOW ? LOW : HIGH;
const int BRAKE_RELAY_OFF = BRAKE_RELAY_ACTIVE_LOW ? HIGH : LOW;

// 두 번째 테스트 코드의 A0 입력 전압 기준을 그대로 적용.
// 두 임계값 사이에서는 이전 상태를 유지 (히스테리시스).
const float BRAKE_ON_THRESHOLD_V  = 0.4801f;
const float BRAKE_OFF_THRESHOLD_V = 0.4799f;
const float BRAKE_ADC_REFERENCE_V = 3.3f;    // Arduino Due ADC 입력 범위
const float BRAKE_ADC_MAX = 4095.0f;        // analogReadResolution(12)에 대응

int brakePressureAdc = 0;
float brakePressureVoltage = 0.0f;
bool brakeLightOn = false;

constexpr float SENSOR_MAX_MM = 100.0f;
const int   VAL_MIN = 0;
const int   VAL_MAX = 4095;

// ================= [사용자가 조정할 주요 파라미터] =================
// A1 센서의 절대 위치(mm). 숫자가 작아지는 방향이 브레이크 당김.
// "거리"는 이동량이 아닌 목표 위치이며, 실제 이동량 = 풀림 위치 - 당김 위치.
// 최종 첫 번째 코드의 90/35mm를 유지 (두 번째 테스트 코드는 75/35mm).
constexpr float POS_RELEASE = 90.0f;  // [수정] 최대 풀림 위치(mm): READY/AUTO 공통
constexpr float POS_FULL = 35.0f;     // [수정] 최대 당김 위치(mm): PC/비상정지 공통
constexpr float PC_FIRST_RATIO = 0.70f; // [수정] 풀림~당김 이동 구간의 70% (0~1)
const unsigned long PC_HOLD_MS = 300UL; // [수정] 중간 위치 유지 시간(ms), 300 = 0.3초
// 두 번째 파일은 HOLD_MS=200이나, 이번 요청의 0.3초를 적용.
const float POS_FIRST = POS_RELEASE + (POS_FULL - POS_RELEASE) * PC_FIRST_RATIO;
// 기본값: 90 + (35 - 90) * 0.70 = 51.5mm. 끝점 변경 시 자동 재계산.
const float POS_READY = POS_RELEASE;
const float POS_AUTO = POS_RELEASE;
const float POS_BRAKE = POS_FULL;
const float POS_EMERGENCY = POS_FULL;
const unsigned long MOTOR_REVERSE_PAUSE_MS = 10UL; // [수정] 방향 반전 보호 정지 시간
const unsigned long PRINT_INTERVAL_MS = 200UL;    // [수정] 상태 출력 주기
static_assert(POS_FULL >= 0 && POS_FULL < POS_RELEASE && POS_RELEASE <= SENSOR_MAX_MM,
              "Check brake travel endpoints");
static_assert(PC_FIRST_RATIO > 0 && PC_FIRST_RATIO < 1, "Check first-stage ratio");

// [수정] 목표 도달 허용 오차(mm). 중간 단계도 이 오차 이내에서 도달로 판단.
const float GAP_MM = 1.5f;

// [수정] 센서 필터 계수(0~1). 작을수록 평활화가 강하고 응답이 느려짐.
const float FILTER_ALPHA = 0.15f;

// ================= [타이머 설정] =================
// 긴급정지는 mode == 3 자체로 판단하므로 EMG timeout 사용하지 않음
// [수정] PC 무수신 시 브레이크 해제까지의 시간(ms). 기존 정책 유지.
// ASMS mode 3에서는 PC 타임아웃과 무관하게 비상정지 유지.
const unsigned long PC_TIMEOUT_MS = 20000;

// ================= [네트워크 설정] =================
IPAddress localIp(10, 177, 21, 3);
IPAddress asmsIP(10, 177, 21, 5);   // ASMS
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x21, 0x00 };
unsigned int localPort = 5000;

EthernetUDP Udp;
uint8_t rxBuf[32];

// ================= [시스템 상태 변수] =================
uint8_t receivedMode = 0;
int Rbreak = 0;

unsigned long lastPcRx = 0;
unsigned long lastPrintTime = 0;

// 센서 필터용 변수
float filteredPos = 70.0f;
bool filterInit = false;

// 모터 상태
enum MotorState { STOP, PUSH, PULL };
MotorState lastMotorState = STOP;
MotorState lastDrivenDirection = STOP;
unsigned long motorStoppedAt = 0;

enum PcBrakeStage { PC_IDLE, PC_APPROACH_70, PC_HOLD_70, PC_FULL_BRAKE };
PcBrakeStage pcBrakeStage = PC_IDLE;
unsigned long pcHoldStart = 0;

// PC brake=1이 유지되는 동안 한 번만 진행. 반복 수신해도 재시작하지 않음.
// brake=0 / 모드 변경 / PC 타임아웃 / 비상정지 때 초기화.
float selectTarget(float current, unsigned long now) {
  if (receivedMode == 3) { // 최우선: 70% 단계와 유지 시간을 즉시 취소
    pcBrakeStage = PC_IDLE;
    return POS_EMERGENCY;
  }
  if (receivedMode != 1 || Rbreak != 1) {
    pcBrakeStage = PC_IDLE;
    return receivedMode == 1 ? POS_AUTO : POS_READY;
  }
  if (pcBrakeStage == PC_IDLE) pcBrakeStage = PC_APPROACH_70;
  if (pcBrakeStage == PC_APPROACH_70 && current <= POS_FIRST + GAP_MM) {
    pcBrakeStage = PC_HOLD_70;
    pcHoldStart = now; // 최초 도달 때만 시작: 위치 흔들림으로 재시작하지 않음
  }
  if (pcBrakeStage == PC_HOLD_70 && now - pcHoldStart >= PC_HOLD_MS) {
    pcBrakeStage = PC_FULL_BRAKE;
  }
  // 두 번째 t 코드와 동일: 유지 중 풀림 방향으로 밀리면 중간 목표로 다시 당김.
  // 사람이 더 밟았을 때는 밀어내지 않음. delay(300)을 사용하지 않음.
  return pcBrakeStage == PC_FULL_BRAKE ? POS_BRAKE : POS_FIRST;
}

// ================= [모터 제어 기본 함수] =================
void motorStop() {
  if (lastMotorState != STOP) motorStoppedAt = millis();
  digitalWrite(PIN_MT_FWD, LOW);
  digitalWrite(PIN_MT_BWD, LOW);
  lastMotorState = STOP;
}

void motorPushFull() {
  digitalWrite(PIN_MT_FWD, HIGH);
  digitalWrite(PIN_MT_BWD, LOW);
  lastMotorState = PUSH;
  lastDrivenDirection = PUSH;
}

void motorPullFull() {
  digitalWrite(PIN_MT_FWD, LOW);
  digitalWrite(PIN_MT_BWD, HIGH);
  lastMotorState = PULL;
  lastDrivenDirection = PULL;
}

// ================= [센서 읽기 + 필터링] =================
float readFilteredPosition() {
  int rawVal = analogRead(PIN_SENSOR);

  float rawPos =
      ((float)(rawVal - VAL_MIN) * SENSOR_MAX_MM)
      / (float)(VAL_MAX - VAL_MIN);

  if (!filterInit) {
    filteredPos = rawPos;
    filterInit = true;
  }

  filteredPos =
      filteredPos * (1.0f - FILTER_ALPHA)
      + rawPos * FILTER_ALPHA;

  return filteredPos;
}

// ================= [압력센서 기반 브레이크등 제어] =================
// 모터/UDP 명령과는 독립적으로 실제 압력센서 신호로 릴레이를 구동함.
// delay()를 사용하지 않아 통신 및 모터 제어 주기를 방해하지 않음.
void updateBrakeLight() {
  brakePressureAdc = analogRead(PIN_BRAKE_PRESSURE);
  brakePressureVoltage =
      (float)brakePressureAdc * BRAKE_ADC_REFERENCE_V / BRAKE_ADC_MAX;

  if (brakePressureVoltage >= BRAKE_ON_THRESHOLD_V) {
    brakeLightOn = true;
  } else if (brakePressureVoltage <= BRAKE_OFF_THRESHOLD_V) {
    brakeLightOn = false;
  }

  digitalWrite(PIN_BRAKE_RELAY,
               brakeLightOn ? BRAKE_RELAY_ON : BRAKE_RELAY_OFF);
}

// ================= [개선된 모터 제어 함수] =================
// allowPush = true
//   목표보다 더 들어갔을 때 다시 밀어서 복귀 가능
//
// allowPush = false
//   목표보다 더 들어갔을 때 밀어내지 않고 정지
void controlMotor(float current, float target, bool allowPush) {
  const float error = current - target;
  MotorState desired = STOP;
  if (error > GAP_MM) desired = PULL;
  else if (error < -GAP_MM && allowPush) desired = PUSH;

  if (desired == STOP) { motorStop(); return; }
  // 방향 반전 시 보호 정지는 유지하되, delay 없이 UDP를 계속 처리.
  // 비상정지 목표는 즉시 반영되지만, 반전 중에는 최대 10ms 보호 정지가 적용됨.
  if (lastDrivenDirection != STOP && desired != lastDrivenDirection) {
    if (lastMotorState != STOP) motorStop();
    if (millis() - motorStoppedAt < MOTOR_REVERSE_PAUSE_MS) return;
  }
  if (desired == PULL) motorPullFull();
  else motorPushFull();
}

// ================= [초기화] =================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(PIN_MT_FWD, OUTPUT);
  pinMode(PIN_MT_BWD, OUTPUT);

  // 릴레이는 부팅 시 OFF로 초기화
  digitalWrite(PIN_BRAKE_RELAY, BRAKE_RELAY_OFF);
  pinMode(PIN_BRAKE_RELAY, OUTPUT);

  // Ethernet Shield의 SD 카드 칩 선택 비활성화 (D4는 릴레이에 사용 금지)
  digitalWrite(4, HIGH);
  pinMode(4, OUTPUT);

  motorStop();

  Ethernet.begin(mac, localIp);
  Udp.begin(localPort);

  lastPcRx = millis();

  Serial.println(
      "=== Brake Board ROBUST + HUMAN BRAKE PROTECTION ==="
  );
}

// ================= [메인 루프] =================
void loop() {

  // ---------------------------------------------------------
  // 1. 센서 데이터 읽기 + 필터링
  // ---------------------------------------------------------
  float currentPos = readFilteredPosition();

  // 브레이크등: A0 압력센서 0.4801V 이상 ON, 0.4799V 이하 OFF (D5 릴레이)
  updateBrakeLight();

  // ---------------------------------------------------------
  // 2. UDP 통신 수신
  // ---------------------------------------------------------
  int packetSize = Udp.parsePacket();

  if (packetSize > 0) {

    IPAddress senderIP = Udp.remoteIP();

    int n =
        Udp.read(
            rxBuf,
            min(packetSize, (int)sizeof(rxBuf))
        );


    // =========================================================
    // ASMS 제어기 패킷 처리 (5 bytes)
    // =========================================================
    if (
        packetSize == 5 &&
        senderIP == asmsIP &&
        n >= 1
    ) {

      receivedMode = rxBuf[0];





      // -------------------------------------------------------
      // 긴급정지
      // -------------------------------------------------------
      if (receivedMode == 3) {

        // mode 3 자체를 긴급정지 상태로 사용


        // 긴급정지에서는 PC brake 값 무시
        Rbreak = 0;
      }

      else {

        // 자율주행 모드가 아니면
        // PC 브레이크 강제 해제
        if (receivedMode != 1) {
          Rbreak = 0;
        }
      }
    }


    // =========================================================
    // PC 제어 명령 패킷 처리 (9 bytes)
    // =========================================================
    else if (
        packetSize == 9 &&
        n >= 9
    ) {

      // 자율주행 모드일 때만
      // PC brake 명령 인정
      if (receivedMode == 1) {

        uint8_t misc = rxBuf[8];

        uint8_t brakeBit =
            (misc >> 7) & 0x01;


        // MODE 1일 때만 현재 PC brake 값을 즉시 반영
        Rbreak = brakeBit;


        lastPcRx = millis();
      }

      else {

        // MODE 1이 아니면
        // PC brake 명령 무시
        Rbreak = 0;
      }
    }
  }


  // ---------------------------------------------------------
  // 3. PC 통신 타임아웃
  // ---------------------------------------------------------
  if (
      millis() - lastPcRx >
      PC_TIMEOUT_MS
  ) {

    Rbreak = 0;
  }


  // =========================================================
  // 4. 현재 상태에 따른 목표 위치 결정
  // =========================================================
  // PC 전용 단계 제어보다 ASMS 비상정지를 먼저 검사한다.
  const float targetPos = selectTarget(currentPos, millis());

  // =========================================================
  // 5. 사람 브레이크 보호 로직
  // =========================================================

  // MODE 3 긴급정지 또는
  // MODE 1에서 PC Brake ON이면
  // 브레이크가 밟힌 상태
  bool isBrakeTarget =
      (receivedMode == 3) ||
      (receivedMode == 1 && Rbreak == 1);


  // 브레이크/긴급정지 상태에서는
  // 목표보다 더 밟혀도 다시 밀지 않음
  bool allowPush =
      !isBrakeTarget;


  // 모터 제어
  controlMotor(
      currentPos,
      targetPos,
      allowPush
  );


  // =========================================================
  // 6. 시리얼 디버그 출력
  // =========================================================
  if (
      millis() - lastPrintTime >
      PRINT_INTERVAL_MS
  ) {

    lastPrintTime = millis();


    Serial.print("mode=");
    Serial.print(receivedMode);


    Serial.print(" Rbreak=");
    Serial.print(Rbreak);


    Serial.print(" pos=");
    Serial.print(currentPos, 1);


    Serial.print(" tgt=");
    Serial.print(targetPos, 1);


    Serial.print(" allowPush=");
    Serial.print(
        allowPush ? 1 : 0
    );

    Serial.print(" brakeADC=");
    Serial.print(brakePressureAdc);

    Serial.print(" brakeV=");
    Serial.print(brakePressureVoltage, 3);

    Serial.print(" brakeLED=");
    Serial.print(brakeLightOn ? "ON" : "OFF");

    Serial.print(" relayD5=");
    Serial.print(brakeLightOn ? BRAKE_RELAY_ON : BRAKE_RELAY_OFF);

    Serial.print(" pcStage=");
    if (pcBrakeStage == PC_APPROACH_70) Serial.print("APPROACH_70");
    else if (pcBrakeStage == PC_HOLD_70) Serial.print("HOLD_70");
    else if (pcBrakeStage == PC_FULL_BRAKE) Serial.print("FULL_BRAKE");
    else Serial.print("IDLE");
    Serial.print(" state=");


    if (receivedMode == 3) {

      Serial.println("EMG");
    }

    else if (Rbreak == 1) {

      Serial.println("BRAKE");
    }

    else if (receivedMode == 1) {

      Serial.println("AUTO");
    }

    else {

      Serial.println("READY");
    }
  }
}