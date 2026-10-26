/* Serial.println은 테스트용이므로 잘 작동하면 주석처리 해주세요.
*/

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// 모터 : {10, 177, 21, 2}, 브레이크 : {10, 177, 21, 3}
IPAddress localIp(10, 177, 21, 2); 
// 아두이노 이더넷 쉴드 뒷면 주소 참고해서 각 0x 뒤에 작성해주세요
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x21, 0x68 }; 

// ASMS IP (변경 X)
IPAddress pcIP(10, 177, 21, 1);
IPAddress motorIP(10, 177, 21, 2);
IPAddress brakeIP(10, 177, 21, 3);
IPAddress steerIP(10, 177, 21, 4);
IPAddress asmsIP(10, 177, 21, 5); 
unsigned int localPort = 5000; // 수신 포트 번호 (변경 X)

EthernetUDP Udp;

uint8_t receivedMode = 0;

void setup() {
  // 115200 고정해주세요!!
  Serial.begin(115200); 

  Ethernet.begin(mac, localIp);
  Udp.begin(localPort);
  
  Serial.println("UDP 수신 준비");
}

void loop() {
  int packetSize = Udp.parsePacket(); 

  if (packetSize > 0) {
    if (packetSize == 5) {
     IPAddress senderIP = Udp.remoteIP();

      // ASMS(5)에서 온 데이터인지 확인
      if (senderIP == asmsIP) {
        uint8_t buffer[5];
        Udp.read(buffer, 5);

        // 데이터 언패킹 (STM32와 순서 일치)
        receivedMode = buffer[0];
        int16_t joy_x = (int16_t)((buffer[2] << 8) | buffer[1]);
        int16_t joy_y = (int16_t)((buffer[4] << 8) | buffer[3]);

        // 파트별 제어 코드 작성
       switch (receivedMode) {
         case 1: // AUTO (자율주행)
          Serial.println("Mode: AUTO");
          /* 자율주행 알고리즘 작성 */
          break;

        case 2: // MANUAL (수동주행)
          Serial.print("MANUAL - X: "); Serial.print(joy_x);
          Serial.print(", Y: "); Serial.println(joy_y);
          /* 조이스틱 값 처리 */
          break;

        case 3: // ESTOP (비상정지)
          Serial.println("Mode: ESTOP");
          /* 안전한 비상정지 구현 */
          break;

        default: // NONE (대기)
          Serial.println("Mode: NONE");
          /* 대기 상태 구현 */
          break;
        }
      }
    }

    else if (packetSize == 9) {
      uint8_t autoBuffer[9];
      Udp.read(autoBuffer, 9);

      int32_t stear = (int32_t)((uint32_t)autoBuffer[0] | ((uint32_t)autoBuffer[1] << 8) | ((uint32_t)autoBuffer[2] << 16) | ((uint32_t)autoBuffer[3] << 24));
      uint32_t speed = (uint32_t)((uint32_t)autoBuffer[4] | ((uint32_t)autoBuffer[5] << 8) | ((uint32_t)autoBuffer[6] << 16) | ((uint32_t)autoBuffer[7] << 24));
      uint8_t misc  = autoBuffer[8];

      // AUTO(1)
      if (receivedMode == 1) {
        Serial.print("[AUTO] 조향각: "); Serial.print(stear);
        Serial.print(" | 속도: "); Serial.print(speed);
        Serial.print(" | 상태(misc): "); Serial.println(misc, BIN);

        /* stear, speed 값을 실제 제어 함수에 넘겨주는 코드 작성 */
        
      } else {
        // 자율주행 모드 X
      }
    }
  } 
} 