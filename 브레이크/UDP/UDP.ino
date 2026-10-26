#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x25, 0x4C }; 
IPAddress localIp(10,177,21,3); // 아두이노의 IP 입력 ( 기존에 쓰던 ip 사용하면됨 )
IPAddress pc(10,177,21,1); // PC(용하님컴퓨터) 의 IP 입력
IPAddress asms(10,177,21,5); // ASMS정보를 뿌려주는 보드의 IP

unsigned int localPort = 5000;  // 수신할 포트

struct MyPacket {
  uint32_t stear; //4byte int
  uint32_t speed;
  uint8_t misc; //1byte
}receivedData;

uint32_t Rstear;
uint32_t Rspeed;
int Rbreak;
int Rasms;

char packetBuffer1[UDP_TX_PACKET_MAX_SIZE]; //일단 고정      // 수신 데이터 버퍼

EthernetUDP Udp;

void setup() {
  Serial.begin(9600);

  // Ethernet 시작
  Ethernet.begin(mac, localIp);
  delay(1000);

  // UDP 포트 열기
  Udp.begin(localPort);

  Serial.println("UDP Receiver Ready");
}

void loop() {
  int packetSize = Udp.parsePacket(); //패킷 조사후 패킷사이즈 반환
  if (packetSize > 0) {
    // 현재 데이터를 주는 곳의 IP, PORT 확인.
    IPAddress senderIP = Udp.remoteIP();
    unsigned int senderPort = Udp.remotePort();

    //IP주소가 ASMS의 주소이면 ASMS데이터 저장
    //IP주소가 PC의 주소이면 구조체 데이터 저장
    if(senderIP ==  asms){
      uint8_t asms=0;
      Udp.read(&asms,1);
      Serial.print("asms 데이터 수신 : ");
      Serial.println(asms);
      Rasms = asms; //ASMS 데이터 저장.
    }
    else if(senderIP == pc){
      Udp.read(packetBuffer1, sizeof(receivedData)); // PC에서 보낸 데이터 읽기
      Serial.print("Received: "); 
      memcpy(&receivedData, packetBuffer1, sizeof(receivedData)); //packetBuffer에 데이터 복사
      Rstear = receivedData.stear;
      Rspeed = receivedData.speed;
      Rbreak = (receivedData.misc >> 7); //브레이크 데이터 저장
      //출력예시
      Serial.print("조향값: ");
      Serial.println(Rstear); //조향값 출력
      Serial.print("속도값: ");
      Serial.println(Rspeed); //속도값 출력
      Serial.print("브레이크: ");
      Serial.println(Rbreak); //브레이크값 출력
      Serial.print("ASMS: ");
      Serial.println(Rasms);
    }    
  }
}