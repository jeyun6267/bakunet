
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x0C, 0x66 };

// 고정 IP 설정
IPAddress ip(10, 177, 21, 3);
unsigned int localport = 5000;
IPAddress pcIP(10, 177, 21, 1);
IPAddress asmsIP(10, 177, 21, 5);
unsigned int pcPort = 5000;

char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
EthernetUDP Udp;

#pragma pack(push, 1)
struct MyPacket {
  uint32_t stear;
  uint32_t speed;
  uint8_t misc;
} receivedData;
#pragma pack(pop)

int RPWM_Output = 5; // 역회전 핀
int LPWM_Output = 6; // 정회전 핀

bool brake_value = 0;
uint8_t Rasms = 255; // 초기값 255 (비상정지 아님)

// ===============================
// [ADD] 2초 제한 역회전 동작용 상태 변수/도우미
// ===============================
bool brake_window_active = false;        // 현재 2초 동작창 가동 중인지
unsigned long brake_started_at = 0;      // 2초 시작 시각(ms)
bool prev_brake_value = 0;               // 상승 에지(0→1) 검출용
const uint8_t DUTY_REV  = 255;           // 역회전 듀티(원 코드의 190 유지)
const uint8_t DUTY_FWD  = 255;           // 정회전 듀티(원 코드의 255 유지)

inline void stopMotor() {
  analogWrite(LPWM_Output, 0);
  analogWrite(RPWM_Output, 0);
}
// ===============================

void setup() {
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);

  Serial.begin(9600);
  Ethernet.begin(mac, ip);
  Udp.begin(localport);
  delay(1500);
}

void loop() {
  IPAddress senderIP;
  int pktSize;
  bool gotNew = false;
  MyPacket latest;

  // UDP 수신
  while ((pktSize = Udp.parsePacket()) > 0) {
    senderIP = Udp.remoteIP();

    if (senderIP == pcIP && pktSize == sizeof(MyPacket)) {
      Udp.read((char*)&latest, sizeof(MyPacket));
      gotNew = true;
    }
    else if (senderIP == asmsIP && pktSize == 1) {
      uint8_t asms_byte;
      Udp.read((char*)&asms_byte, 1);

      if (Rasms != asms_byte) {
        Rasms = asms_byte;
        Serial.print("ASMS 값 갱신됨 → ");
        Serial.println(Rasms);
      }

      if (Rasms == 0) {
        brake_value = 1;  // 비상 정지
        Serial.println("비상정지 조건 감지됨 → brake_value = 1 (역회전)");
      }
      else {
        // 복구되면 PC에서 받은 데이터 기반으로 brake_value 재계산
        brake_value = receivedData.misc & (1 << 7);
        Serial.print("비상정지 해제됨 → brake_value 재계산 = ");
        Serial.println(brake_value);
      }
    }
    else {
      Udp.flush();  // 다른 IP에서 온 잘못된 패킷 제거
    }
  }

  // PC에서 온 데이터가 있을 때 출력
  if (gotNew) {
    receivedData = latest;

    Serial.println("===== PC로부터 수신한 데이터 =====");
    Serial.print("stear: ");
    Serial.println(receivedData.stear);
    Serial.print("speed: ");
    Serial.println(receivedData.speed);
    Serial.print("misc (raw): ");
    Serial.println(receivedData.misc, BIN);
    Serial.println("===============================");

    if (Rasms != 0) {
      brake_value = receivedData.misc & (1 << 7);  // 비상정지 아닐 때만 반영
    }

    Serial.print("최종 brake_value: ");
    Serial.println(brake_value);
  }

  // ============================================================
  // [MOD] 모터 제어: 2초 제한 역회전(브레이크=1) → 이후 정지 유지
  //      브레이크가 0으로 바뀌면 정회전 연속 구동
  // ============================================================

  // 상승 에지(0→1)에서만 2초 창 시작 (연속 1이어도 재트리거 방지)
  if (brake_value && !prev_brake_value) {
    brake_window_active = true;
    brake_started_at = millis();
    Serial.println("브레이크 ON (역회전 시작, 2초 한정)");
  }

  // 브레이크 해제되면 창 종료
  if (!brake_value) {
    brake_window_active = false;
  }

  if (brake_value == 1) {
    // 역회전 명령 상태
    if (brake_window_active) {
      // 2초(여기선 1100ms) 이내면 역회전 구동
      if (millis() - brake_started_at < 700UL) {
        analogWrite(LPWM_Output, DUTY_REV);
        analogWrite(RPWM_Output, 0);
      } else {
        // 2초 경과 → 정지로 전환하고 창 닫기
        brake_window_active = false;
        stopMotor();
        Serial.println("2초 경과 → 모터 정지(브레이크 유지 상태)");
      }
    } else {
      // 2초 창이 끝난 이후에는 계속 정지 유지
      stopMotor();
    }
  } else {
    // 정회전 연속 구동
    analogWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, DUTY_FWD);
  }

  // 에지 검출을 위한 이전값 업데이트
  prev_brake_value = brake_value;
}
