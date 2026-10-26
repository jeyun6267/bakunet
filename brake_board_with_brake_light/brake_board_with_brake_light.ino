/* =========================================================================
   Brake Board (Robust & Anti-Hunting Version + Human Brake Protection)
   - ASMS : mode는 첫 바이트만 신뢰
   - PC   : 자율주행(mode == 1)일 때만 brake bit 신뢰
   - 수동 : mode != 1일 때 PC brake 신호를 항상 무시
   - 모터 : 불감대 + 히스테리시스 + 센서 필터 적용
   - 추가 : 브레이크/긴급정지 중 사람이 브레이크를 더 밟으면 액추에이터가 밀어내지 않음
   - 수정 : mode == 3인 동안 긴급정지 브레이크를 계속 유지
   - 추가 : A0 압력센서 1.0V 이상일 때 D5 릴레이로 브레이크등 점등
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

// 기존 단독 테스트 코드의 HIGH 트리거 릴레이 동작 유지
const int BRAKE_RELAY_ON  = HIGH;
const int BRAKE_RELAY_OFF = LOW;

const float BRAKE_ON_THRESHOLD_V = 1.0f;
const float BRAKE_ADC_REFERENCE_V = 5.0f;   // 실제 보드의 ADC 기준전압 확인 필요
const float BRAKE_ADC_MAX = 4095.0f;        // analogReadResolution(12)에 대응

int brakePressureAdc = 0;
float brakePressureVoltage = 0.0f;
bool brakeLightOn = false;

const float SENSOR_MAX_MM = 100.0f;
const int   VAL_MIN = 0;
const int   VAL_MAX = 4095;

// 목표 위치 (mm 단위)
const float POS_READY     = 90.0f;
const float POS_AUTO      = 90.0f;
const float POS_BRAKE     = 35.0f;
const float POS_EMERGENCY = 35.0f;

// 깔짝임 방지 불감대
const float GAP_MM = 1.5f;

// 센서 필터 계수
const float FILTER_ALPHA = 0.15f;

// ================= [타이머 설정] =================
// 긴급정지는 mode == 3 자체로 판단하므로 EMG timeout 사용하지 않음
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

// ================= [모터 제어 기본 함수] =================
void motorStop() {
  digitalWrite(PIN_MT_FWD, LOW);
  digitalWrite(PIN_MT_BWD, LOW);
  lastMotorState = STOP;
}

void motorPushFull() {
  digitalWrite(PIN_MT_FWD, HIGH);
  digitalWrite(PIN_MT_BWD, LOW);
  lastMotorState = PUSH;
}

void motorPullFull() {
  digitalWrite(PIN_MT_FWD, LOW);
  digitalWrite(PIN_MT_BWD, HIGH);
  lastMotorState = PULL;
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

  brakeLightOn = (brakePressureVoltage >= BRAKE_ON_THRESHOLD_V);
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
  float error = current - target;

  // 목표 근처면 정지
  if (fabs(error) <= GAP_MM) {
    motorStop();
    return;
  }

  if (error > GAP_MM) {
    // current > target
    // 예: current=69, target=18
    // 브레이크를 밟기 위해 당김

    if (lastMotorState == PUSH) {
      motorStop();
      delay(10);
    }

    motorPullFull();
  }

  else if (error < -GAP_MM) {
    // current < target
    // 사람이 브레이크를 목표보다 더 밟은 경우

    if (allowPush) {
      if (lastMotorState == PULL) {
        motorStop();
        delay(10);
      }

      motorPushFull();

    } else {
      // 브레이크 및 긴급정지 상태에서는
      // 사람의 발을 밀어내지 않음
      motorStop();
    }
  }
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

  // 브레이크등: A0 압력센서 >= 1.0V일 때 켬 (D5 릴레이)
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


      Serial.print("ASMS mode = ");
      Serial.println(receivedMode);


      // -------------------------------------------------------
      // 긴급정지
      // -------------------------------------------------------
      if (receivedMode == 3) {

        // mode 3 자체를 긴급정지 상태로 사용
        Serial.println(">> ESTOP ACTIVE");

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


        Serial.print("PC brake bit = ");
        Serial.print(brakeBit);

        Serial.print(" -> Rbreak = ");
        Serial.println(Rbreak);


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
  float targetPos = POS_READY;


  // ---------------------------------------------------------
  // MODE 3 = 긴급정지
  //
  // ASMS에서 mode 3이 계속 들어오는 동안
  // 목표값 POS_EMERGENCY (현재 35mm)까지 브레이크 작동
  // ---------------------------------------------------------
  if (receivedMode == 3) {

    targetPos = POS_EMERGENCY;
  }

  else {

    // -------------------------------------------------------
    // MODE 1에서 PC Brake ON
    // -------------------------------------------------------
    if (Rbreak == 1) {

      targetPos = POS_BRAKE;
    }

    else {

      // -----------------------------------------------------
      // 자율주행 MODE 1
      // -----------------------------------------------------
      if (receivedMode == 1) {

        targetPos = POS_AUTO;
      }

      // -----------------------------------------------------
      // 일반 MODE
      // -----------------------------------------------------
      else {

        targetPos = POS_READY;
      }
    }
  }


  // =========================================================
  // 5. 사람 브레이크 보호 로직
  // =========================================================

  // MODE 3 긴급정지 또는
  // MODE 1에서 PC Brake ON이면
  // 브레이크가 밟힌 상태
  bool isBrakeTarget =
      (receivedMode == 3) ||
      (Rbreak == 1);


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
      200
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
