const int ledPin=8;
const int ledPin2=13;
const int interval1=1000;
const int interval2=500;
int ledState=LOW;
int ledState2=LOW;

unsigned long previousMillis=0;
unsigned long previousMillis2=0;

void setup() {
  pinMode(8,OUTPUT);
  pinMode(13,OUTPUT);
}

void loop() {
  unsigned long currentMillis= millis();

  if (currentMillis-previousMillis >= interval1){
    previousMillis = currentMillis;
    if (ledState==LOW)
      ledState=HIGH;
    else
      ledState=LOW;
    digitalWrite(ledPin,ledState);
  }
  if (currentMillis-previousMillis2 >= interval2){
    previousMillis2 = currentMillis;
    if (ledState2==LOW)
      ledState2=HIGH;
    else
      ledState2=LOW;
    digitalWrite(ledPin2,ledState2);
  }
}
