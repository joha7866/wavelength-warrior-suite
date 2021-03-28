#include <SparkFun_TB6612.h>
#include <Wire.h>

#define I2C_SEC_ADDR 0x69

#define F_PWMA 5
#define F_AIN2 1
#define F_AIN1 0
#define F_STBY 2
#define F_BIN1 3
#define F_BIN2 4
#define F_PWMB 6

#define B_PWMA 10
#define B_AIN2 8
#define B_AIN1 7
#define B_STBY 9
#define B_BIN1 12
#define B_BIN2 13
#define B_PWMB 11

const int F_OFFSETA = 1;
const int F_OFFSETB = 1;
const int B_OFFSETA = 1;
const int B_OFFSETB = 1;

Motor fl_motor = Motor(F_AIN1, F_AIN2, F_PWMA, F_OFFSETA, F_STBY);
Motor fr_motor = Motor(F_BIN1, F_BIN2, F_PWMB, F_OFFSETB, F_STBY);

Motor bl_motor = Motor(B_AIN1, B_AIN2, B_PWMA, B_OFFSETA, B_STBY);
Motor br_motor = Motor(B_BIN1, B_BIN2, B_PWMB, B_OFFSETB, B_STBY);

const int MOTOR_DUTY = 50;

char receive_buffer[9];
char send_buffer[9];
char error_buffer[9];

char received_command = 'x';
char active_command = 'x';

bool ERROR_FLAG = false;
bool DONE_FLAG = true;
bool PROCESS_RECEIVE_FLAG = false;

void setup() {
    //clear buffers
    receive_buffer[0] = '\0';
    send_buffer[0] = '\0';
    error_buffer[0] = '\0';

    //start listening for I2C
    Wire.begin(I2C_SEC_ADDR);
    Wire.onReceive(receive_handler);
    Wire.onRequest(request_handler);
}

void loop() {
    if(PROCESS_RECEIVE_FLAG == true) {
        do_command();
        PROCESS_RECEIVE_FLAG = false;
    }

    if(ERROR_FLAG == true) {
        brake();
    }
}

void receive_handler(int num_bytes) {
    int i = 0;

    //read msg into receive_buffer
    while(Wire.available() && i<8) {
        receive_buffer[i] = Wire.read();
        i++;
    }
    receive_buffer[i] = '\0';
    received_command = receive_buffer[0];

    //service stop immediately
    if(received_command == 'S') {
        brake();
        active_command = received_command;
        DONE_FLAG = true;
    }
    else if(received_command == '.') {
        //do nothing
    }
    else {
        if(PROCESS_RECEIVE_FLAG == true){
            PROCESS_RECEIVE_FLAG = false;
            ERROR_FLAG = true;
            strcpy(error_buffer,"!BUSY");
        }
        else {
            PROCESS_RECEIVE_FLAG = true;
        }
    }
}

void request_handler() {
    int i = 0;

    if (ERROR_FLAG == true) {
        strcpy(send_buffer, error_buffer);
    }
    else {
        send_buffer[0] = active_command;
        send_buffer[1] = DONE_FLAG?'D':'W';
        send_buffer[2] = '\0';
    }

    while(send_buffer[i] != '\0') {
        Wire.write(send_buffer[i]);
        i++;
    }
}

void do_command() {
    if(received_command == 'F') {
        active_command = received_command;
        drive_forward(MOTOR_DUTY);
        DONE_FLAG = true;
    }
    else if(received_command == 'L') {
        active_command = received_command;
        rotate_left(MOTOR_DUTY);
        DONE_FLAG = true;
    }
    else if(received_command == 'R') {
        active_command = received_command;
        rotate_right(MOTOR_DUTY);
        DONE_FLAG = true;
    }
    else if(received_command == 'E') {
        active_command = received_command;
        ERROR_FLAG = false;
        DONE_FLAG = true;
    }
    else {
        ERROR_FLAG = true;
        strcpy(error_buffer,"!BADCMD");
    }
}

void demo_action() {
    for(int i=0; i<4; i++){
        drive_forward(100);
        delay(500);
        brake();
        delay(500);
        drive_backward(100);
        delay(500);
        brake();
        delay(500);
    }
}

void start_single_motor(int motor_id, int dir, int pwm) {
    switch(motor_id) {
        case 0: // fl
            fl_motor.drive(pwm*dir);
            break;
        case 1: // fr
            fr_motor.drive(pwm*dir);
            break;
        case 2: // bl
            bl_motor.drive(pwm*dir);
            break;
        case 3: // br
            br_motor.drive(pwm*dir);
            break;
        default:
            //do nothing
            break;
    }
}

void stop_single_motor(int motor_id) {
    switch(motor_id) {
        case 0: // fl
            fl_motor.brake();
            break;
        case 1: // fr
            fr_motor.brake();
            break;
        case 2: // bl
            bl_motor.brake();
            break;
        case 3: // br
            br_motor.brake();
            break;
        default:
            //do nothing
            break;
    }
}

void drive_forward(int duty) {
  int pwm = map(duty,0,100,0,255);
  forward(fl_motor,fr_motor,pwm);
  forward(bl_motor,br_motor,pwm);
}

void drive_backward(int duty) {
  int pwm = map(duty,0,100,0,255);
  back(fl_motor,fr_motor,pwm);
  back(bl_motor,br_motor,pwm);
}

void brake() {
  brake(fl_motor,fr_motor);
  brake(bl_motor,br_motor);
}

void translate_right(int duty) {
    int pwm = map(duty,0,100,0,255);
    fl_motor.drive(pwm);
    fr_motor.drive(-pwm);
    bl_motor.drive(-pwm);
    br_motor.drive(pwm);
}

void translate_left(int duty) {
    int pwm = map(duty,0,100,0,255);
    fl_motor.drive(-pwm);
    fr_motor.drive(pwm);
    bl_motor.drive(pwm);
    br_motor.drive(-pwm);
}

// diagonal nw: [1,0,0,1]
// diagonal sw: [0,-1,-1,0]
// diagonal se: [-1,0,0,-1]
// diagonal ne: [0,1,1,0]

// rotate about center cw: [1,-1,1,-1] or ccw: [-1,1,-1,1]
// rotate about back center cw: [1,-1,0,0]
// totate about br wheel cw: [1,0,1,0]

void rotate_left(int duty) {
    int pwm = map(duty,0,100,0,255);
    fl_motor.drive(-pwm);
    fr_motor.drive(pwm);
    bl_motor.drive(-pwm);
    br_motor.drive(pwm);
}

void rotate_right(int duty) {
    int pwm = map(duty,0,100,0,255);
    fl_motor.drive(pwm);
    fr_motor.drive(-pwm);
    bl_motor.drive(pwm);
    br_motor.drive(-pwm);
}
