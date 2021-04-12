#include <Wire.h>

#define SECONDARY_ADDR 0x53
#define POP_PIN 4
#define TARGET_PIN 3
int DANGER_LAZER = 12;
int TARG_LAZER = 11;

char receive_buffer[32];
bool POP_FLAG = false;

void setup() {
  receive_buffer[0] = '\0';
  pinMode(DANGER_LAZER, OUTPUT);
  pinMode(TARG_LAZER, OUTPUT);
  digitalWrite(DANGER_LAZER, LOW);
  digitalWrite(TARG_LAZER, LOW);
  Wire.begin(SECONDARY_ADDR);
  Wire.onReceive(receive_handler);
  Wire.onRequest(send_handler);
}

void loop() {
  if (POP_FLAG) {
    digitalWrite(TARG_LAZER, HIGH);
    delay(5000);
    digitalWrite(DANGER_LAZER, HIGH);
    delay(5000);
    digitalWrite(DANGER_LAZER, LOW);
    digitalWrite(TARG_LAZER, LOW);
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
