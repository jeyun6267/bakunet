/* 
Serial.println은 테스트용이므로 잘 작동하면 주석처리 해주세요.
*/

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// 모터 : {10, 177, 21, 2}, 브레이크 : {10, 177, 21, 3}
IPAddress localIp(10, 177, 21, 0); 
// 아두이노 이더넷 쉴드 뒷면 주소 참고해서 각 0x 뒤에 작성해주세요
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x21, 0x00 }; 

// ASMS IP (변경 X)
IPAddress asmsIP(10, 177, 21, 5); 
unsigned int localPort = 5000; // 수신 포트 번호 (변경 X)

EthernetUDP Udp;

void setup() {
  // 115200 고정해주세요!!
  Serial.begin(115200); 

  Ethernet.begin(mac, localIp);
  Udp.begin(localPort);
  
  Serial.println("UDP 수신 준비");
}

void loop() {
  int packetSize = Udp.parsePacket(); 
  
  if (packetSize >= 5) {
    IPAddress senderIP = Udp.remoteIP();

    // ASMS(5)에서 온 데이터인지 확인
    if (senderIP == asmsIP) {
      uint8_t buffer[5];
      Udp.read(buffer, 5);

     // 데이터 언패킹 (STM32와 순서 일치)
     uint8_t receivedMode = buffer[0];
     int16_t joy_x = (int16_t)((buffer[2] << 8) | buffer[1]);
     int16_t joy_y = (int16_t)((buffer[4] << 8) | buffer[3]);

      // 파트별 제어 코드 작성 (if문으로 바꿔도 됩니다)
      switch (receivedMode) {
        case 1: // AUTO (자율주행)
          Serial.println("Mode: AUTO");
          /* 
          자율주행 알고리즘 작성
          */
          break;

        case 2: // MANUAL (수동주행)
          Serial.print("MANUAL - X: "); Serial.print(joy_x);
          Serial.print(", Y: "); Serial.println(joy_y);
          /* 
          조이스틱 값 처리
          x축 (-2048 ~ +2047): 앞뒤 / y축(-2048 ~ +2047): 좌우
          */
          break;

        case 3: // ESTOP (비상정지)
          Serial.println("Mode: ESTOP");
          /* 
          안전한 비상정지 구현
          */
          break;

        default: // NONE (대기)
          Serial.println("Mode: NONE");
          /* 
          대기 상태 구현
          */
          break;
      }
    }
  }
}