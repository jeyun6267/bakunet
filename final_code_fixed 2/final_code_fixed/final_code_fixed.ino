/* =========================================================================
   Brake Board (Robust & Anti-Hunting Version + Human Brake Protection)
   - ASMS : mode는 첫 바이트만 신뢰
   - PC   : 자율주행(mode == 1)일 때만 brake bit 신뢰
   - 모터 : 불감대 + 히스테리시스 + 센서 필터 적용
   - 추가 : 브레이크/긴급정지 중 사람이 브레이크를 더 밟으면 액추에이터가 밀어내지 않음
   ========================================================================= */

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <math.h>

// ================= [센서/모터 핀 및 설정] =================
const int PIN_SENSOR = A1;
const int PIN_MT_FWD = 8;   // rpwm
const int PIN_MT_BWD = 7;   // lpwm

const float SENSOR_MAX_MM = 100.0f;
const int   VAL_MIN = 0;
const int   VAL_MAX = 4095;

// 목표 위치 (mm 단위)
// READY/AUTO = 브레이크 안 밟힘
// BRAKE/EMERGENCY = 70 -> 50, 즉 20mm = 2cm 수축
const float POS_READY     = 69.0f;
const float POS_AUTO      = 40.0f;
const float POS_BRAKE     = 18.0f;
const float POS_EMERGENCY = 18.0f;

// 깔짝임 방지 불감대
const float GAP_MM = 1.5f;

// 센서 필터 계수
// 작을수록 부드럽지만 반응이 느림
// 0.10~0.20 사이 추천
const float FILTER_ALPHA = 0.15f;

// ================= [타이머 설정] =================
const unsigned long EMG_TIMEOUT_MS = 500;
const unsigned long PC_TIMEOUT_MS  = 20000;

// ================= [네트워크 설정] =================
IPAddress localIp(10, 177, 21, 3);
IPAddress asmsIP(10, 177, 21, 5);   // PC
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x21, 0x00 };
unsigned int localPort = 5000;

EthernetUDP Udp;
uint8_t rxBuf[32];

// ================= [시스템 상태 변수] =================
uint8_t receivedMode = 0;
int Rbreak = 0;
bool isEmergency = false;

// 모드 1 진입 직후에는 이전에 걸려 있던 brake ON 신호를 무시하고,
// PC가 brake OFF(bit=0)를 한 번 보낸 뒤부터 ON을 인정한다.
bool waitBrakeRelease = false;

unsigned long lastAsmsRx = 0;
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

  float rawPos = ((float)(rawVal - VAL_MIN) * SENSOR_MAX_MM) / (float)(VAL_MAX - VAL_MIN);

  if (!filterInit) {
    filteredPos = rawPos;
    filterInit = true;
  }

  filteredPos = filteredPos * (1.0f - FILTER_ALPHA) + rawPos * FILTER_ALPHA;

  return filteredPos;
}

// ================= [개선된 모터 제어 함수] =================
// allowPush = true  : 목표보다 더 들어갔을 때 다시 밀어서 복귀 가능
// allowPush = false : 목표보다 더 들어갔을 때 밀어내지 않고 정지
void controlMotor(float current, float target, bool allowPush) {
  float error = current - target;

  // 목표 근처면 정지
  if (fabs(error) <= GAP_MM) {
    motorStop();
    return;
  }

  if (error > GAP_MM) {
    // current > target
    // 예: current=70, target=50
    // 브레이크를 밟기 위해 당김
    if (lastMotorState == PUSH) {
      motorStop();
      delay(10);
    }

    motorPullFull();
  }
  else if (error < -GAP_MM) {
    // current < target
    // 예: target=50인데 사람이 발로 더 밟아서 current=45
    // 브레이크 상태에서는 액추에이터가 발을 밀어내면 안 됨
    if (allowPush) {
      if (lastMotorState == PULL) {
        motorStop();
        delay(10);
      }

      motorPushFull();
    } else {
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
  motorStop();

  Ethernet.begin(mac, localIp);
  Udp.begin(localPort);

  lastPcRx = millis();
  lastAsmsRx = millis();

  Serial.println("=== Brake Board ROBUST + HUMAN BRAKE PROTECTION ===");
}

// ================= [메인 루프] =================
void loop() {

  // 1. 센서 데이터 읽기 + 필터링
  float currentPos = readFilteredPosition();

  // 2. UDP 통신 수신 및 필터링
  int packetSize = Udp.parsePacket();

  if (packetSize > 0) {
    IPAddress senderIP = Udp.remoteIP();
    int n = Udp.read(rxBuf, min(packetSize, (int)sizeof(rxBuf)));

    // ===== ASMS 제어기 패킷 처리 (5 bytes) =====
    if (packetSize == 5 && senderIP == asmsIP && n >= 1) {
      uint8_t newMode = rxBuf[0];

      // 모드 1로 "새로" 진입하는 순간:
      // 이전 모드에서 계속 날아오던 brake ON 패킷이 바로 적용되지 않도록
      // brake OFF를 한 번 받을 때까지 ON을 무시하는 상태로 전환
      if (newMode == 1 && receivedMode != 1) {
        waitBrakeRelease = true;
        Rbreak = 0;
        Serial.println(">> MODE1 ENTER: waiting brake release");
      }

      receivedMode = newMode;

      Serial.print("ASMS mode = ");
      Serial.println(receivedMode);

      if (receivedMode == 3) {
        isEmergency = true;
        lastAsmsRx = millis();
        Serial.println(">> ESTOP ACTIVE");
      } else {
        isEmergency = false;

        // 자율주행 모드가 아니면 PC 브레이크 강제 해제
        if (receivedMode != 1) {
          Rbreak = 0;
        }
      }
    }

    // ===== PC 제어 명령 패킷 처리 (9 bytes) =====
    else if (packetSize == 9 && n >= 9) {

      // 자율주행 모드일 때만 PC brake 인정
      if (receivedMode == 1) {
        uint8_t misc = rxBuf[8];
        uint8_t brakeBit = (misc >> 7) & 0x01;

        if (waitBrakeRelease) {
          // 모드 1 진입 후 아직 OFF를 못 봤음
          // -> ON은 이전 모드에서 남은 신호로 간주하고 무시
          if (brakeBit == 0) {
            waitBrakeRelease = false;
            Serial.println(">> brake release seen, PC brake enabled");
          }
          Rbreak = 0;
        } else {
          Rbreak = brakeBit;
        }

        Serial.print("PC brake bit = ");
        Serial.print(brakeBit);
        Serial.print(" -> Rbreak = ");
        Serial.println(Rbreak);

        lastPcRx = millis();
      } else {
        Rbreak = 0;
      }
    }
  }

  // 3. 타임아웃 예외 처리
  if (millis() - lastPcRx > PC_TIMEOUT_MS) {
    Rbreak = 0;
  }

  if (isEmergency && (millis() - lastAsmsRx > EMG_TIMEOUT_MS)) {
    isEmergency = false;
    Serial.println(">> ESTOP RELEASED");
  }

  // 4. 현재 상태에 따른 목표 위치 결정
  float targetPos = POS_READY;

  if (isEmergency) {
    targetPos = POS_EMERGENCY;
  } else {
    if (Rbreak == 1) {
      targetPos = POS_BRAKE;
    } else {
      if (receivedMode == 1) {
        targetPos = POS_AUTO;
      } else {
        targetPos = POS_READY;
      }
    }
  }

  // 5. 사람이 브레이크를 더 밟았을 때 밀어내지 않도록 제어
  bool isBrakeTarget = isEmergency || (Rbreak == 1);

  // 브레이크/긴급정지 상태에서는 push 금지
  // 즉, 목표보다 더 밟힌 상태에서는 액추에이터가 정지함
  bool allowPush = !isBrakeTarget;

  controlMotor(currentPos, targetPos, allowPush);

  // 6. 시리얼 디버그 출력
  if (millis() - lastPrintTime > 200) {
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
    Serial.print(allowPush ? 1 : 0);

    Serial.print(" state=");
    if (isEmergency) {
      Serial.println("EMG");
    } else if (Rbreak) {
      Serial.println("BRAKE");
    } else if (receivedMode == 1) {
      Serial.println("AUTO");
    } else {
      Serial.println("READY");
    }
  }
}