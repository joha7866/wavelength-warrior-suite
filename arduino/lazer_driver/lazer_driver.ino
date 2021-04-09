#include <Wire.h>

#define SECONDARY_ADDR 0x53
#define POP_PIN 4
#define TARGET_PIN 3

char receive_buffer[32];
bool POP_FLAG = false;

void setup() {
  receive_buffer[0] = '\0';
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(3, LOW);
  Wire.begin(SECONDARY_ADDR);
  Wire.onReceive(receive_handler);
  Wire.onRequest(send_handler);
}

void loop() {
  if (POP_FLAG) {
    digitalWrite(3, HIGH);
    delay(5000);
    digitalWrite(4, HIGH);
    delay(5000);
    digitalWrite(4, LOW);
    digitalWrite(3, LOW);
    POP_FLAG = false;
  }
}

void receive_handler(int how_many)
{
  int i = 0;
  while(Wire.available() && i<31)
  {
    receive_buffer[i] = Wire.read();
    i++;
  }
  
  receive_buffer[i] = '\0';

  if(!strcmp(receive_buffer, "POPseq") && POP_FLAG == false) {
    POP_FLAG = true;
  }
}

void send_handler()
{
  Wire.write(0x61);
}
