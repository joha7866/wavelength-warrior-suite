#include <Wire.h>

#define SECONDARY_ADDR 0x53
#define DANGER_LAZER 12
#define TARG_LAZER 11

const int PREFIRE_DELAY = 5000; //5s
const int FIRE_DURATION = 5000; //5s

char receive_buffer[16];
unsigned long cmd_ts;
bool GOOD_TO_GO;
char active_cmd;

void setup() {
    receive_buffer[0] = '\0';
    GOOD_TO_GO = false;
    cmd_ts = 0;
    active_cmd = '\0';

    pinMode(DANGER_LAZER, OUTPUT);
    pinMode(TARG_LAZER, OUTPUT);
    digitalWrite(DANGER_LAZER, LOW);
    digitalWrite(TARG_LAZER, LOW);

    Wire.begin(SECONDARY_ADDR);
    Wire.onReceive(receive_handler);
    Wire.onRequest(request_handler);
}

void loop() {
    if(GOOD_TO_GO) {
        switch(active_cmd) {
        case 'P':
            if(millis() < cmd_ts + PREFIRE_DELAY) {
                digitalWrite(TARG_LAZER, HIGH);
                digitalWrite(DANGER_LAZER, LOW);
            } else if(millis() < cmd_ts + PREFIRE_DELAY + FIRE_DURATION) {
                digitalWrite(TARG_LAZER, HIGH);
                digitalWrite(DANGER_LAZER, HIGH);
            } else {
                digitalWrite(TARG_LAZER, LOW);
                digitalWrite(DANGER_LAZER, LOW);
                GOOD_TO_GO = false;
                cmd_ts = 0;
                active_cmd = '\0';
            }
            break;
        case 'T':
            if(millis() < cmd_ts + PREFIRE_DELAY) {
                digitalWrite(TARG_LAZER, HIGH);
                digitalWrite(DANGER_LAZER, LOW);
            } else if(millis() < cmd_ts + PREFIRE_DELAY + FIRE_DURATION){
                digitalWrite(TARG_LAZER, HIGH);
                digitalWrite(DANGER_LAZER, LOW);
            } else {
                digitalWrite(TARG_LAZER, LOW);
                digitalWrite(DANGER_LAZER, LOW);
                GOOD_TO_GO = false;
                cmd_ts = 0;
                active_cmd = '\0';
            }
            break;
        default:
            break;
        }
    } else {
        digitalWrite(TARG_LAZER, LOW);
        digitalWrite(DANGER_LAZER, LOW);
    }
}

////
// 'P' is pop
// 'T' is test
// all other bytes will cancel
void receive_handler(int how_many) {
    int i = 0;
    while(Wire.available() && i < 16) {
        receive_buffer[i] = Wire.read();
        i++;
    }
    receive_buffer[i] = '\0';

    active_cmd = receive_buffer[0];
    cmd_ts = millis();
    if(active_cmd == 'P' || active_cmd == 'T') {
        GOOD_TO_GO = true;
    } else {
        GOOD_TO_GO = false;
    }
}

void request_handler() {
    Wire.write(active_cmd);
}
