#include <SparkFun_TB6612.h>
#include <Wire.h>

#define I2C_SEC_ADDR 0x69

#define SLEEP_PIN A0

#define F_PWMA 6
#define F_AIN2 4
#define F_AIN1 3
#define F_STBY 2
#define F_BIN1 0
#define F_BIN2 1
#define F_PWMB 5

#define B_PWMA 11
#define B_AIN2 13
#define B_AIN1 12
#define B_STBY 9
#define B_BIN1 7
#define B_BIN2 8
#define B_PWMB 10

const int F_OFFSETA = 1;
const int F_OFFSETB = 1;
const int B_OFFSETA = 1;
const int B_OFFSETB = -1;

Motor fl_motor = Motor(F_AIN1, F_AIN2, F_PWMA, F_OFFSETA, F_STBY);
Motor fr_motor = Motor(F_BIN1, F_BIN2, F_PWMB, F_OFFSETB, F_STBY);

Motor bl_motor = Motor(B_AIN1, B_AIN2, B_PWMA, B_OFFSETA, B_STBY);
Motor br_motor = Motor(B_BIN1, B_BIN2, B_PWMB, B_OFFSETB, B_STBY);

int motor_pwm;

char receive_buffer[9];
char send_buffer[9];
char error_buffer[9];

char cmd_direction[4];
char command_code = 'x';
char active_command = 'x';
char input_pwm = '\0';

bool SLEEP_FLAG = false;
bool ERROR_FLAG = false;
bool DONE_FLAG = true;
bool PROCESS_RECEIVE_FLAG = false;

void setup() {
    //clear buffers
    receive_buffer[0] = '\0';
    send_buffer[0] = '\0';
    error_buffer[0] = '\0';
    cmd_direction[0] = '\0';

    //set default speed at 50%
    motor_pwm = 127;

    //start listening for I2C
    Wire.begin(I2C_SEC_ADDR);
    Wire.onReceive(receive_handler);
    Wire.onRequest(request_handler);
}

void loop() {
    if(analogRead(SLEEP_PIN) < 256) {
        if(SLEEP_FLAG == false) {
            brake_all();
        }
        SLEEP_FLAG = true;
        PROCESS_RECEIVE_FLAG = false;
        ERROR_FLAG = false;
        DONE_FLAG = true;
        active_command = 'x';
    } else {
        SLEEP_FLAG = false;
    }

    if(SLEEP_FLAG == false) {
        if(PROCESS_RECEIVE_FLAG == true) {
            active_command = command_code;
            do_command();
            PROCESS_RECEIVE_FLAG = false;
            DONE_FLAG = true;
        }

        if(ERROR_FLAG == true) {
            brake_all();
        }
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

    //set rxed cmd to first byte in buffer
    command_code = receive_buffer[0];
    if(command_code == 'R'||command_code == 'T'||command_code == 'D') {
        cmd_direction[0] = receive_buffer[1];
        cmd_direction[1] = receive_buffer[2];
        cmd_direction[2] = receive_buffer[3];
    }
    else if(command_code == 'P') {
        input_pwm = receive_buffer[1];
    }

    //service stop cmd immediately
    if(command_code == 'S') {
        brake_all();
        active_command = command_code;
        DONE_FLAG = true;
    }
    //do nothing to service poll cmd
    else if(command_code == '.') {
        //do nothing
    }
    //all other commands are serviced in the loop
    else {
        //if i rx a cmd but am actively processing another, switch to error state
        if(PROCESS_RECEIVE_FLAG == true){
            PROCESS_RECEIVE_FLAG = false;
            ERROR_FLAG = true;
            strcpy(error_buffer,"!BUSY");
        }
        //otherwise, set flag to handle in loop
        else {
            PROCESS_RECEIVE_FLAG = true;
        }
    }
}

void request_handler() {
    int i = 0;

    //if in error state, send the error buffer
    if (ERROR_FLAG == true) {
        strcpy(send_buffer, error_buffer);
    }
    //if in sleep state, send zzz's
    else if(SLEEP_FLAG == true) {
        strcpy(send_buffer, "ZZZ");
    }
    //else return info about the current cmd
    else {
        send_buffer[0] = active_command;
        send_buffer[1] = DONE_FLAG?'D':'W';
        send_buffer[2] = '\0';
    }

    //write the bytes in the send buffer
    while(send_buffer[i] != '\0') {
        Wire.write(send_buffer[i]);
        i++;
    }
}

void do_command() {
    switch(command_code) {
    //Forward CMD
    case 'F':
        drive_forward(motor_pwm);
        break;

    //Backward CMD
    case 'B':
        drive_backward(motor_pwm);
        break;

    //Rotate CMDs
    case 'R':
        //Front/back
        switch(cmd_direction[0]) {
        case 'L':
            rotate_left(motor_pwm);
            break;
        case 'R':
            rotate_right(motor_pwm);
            break;
        case 'F':
            //Left/right
            switch(cmd_direction[1]) {
            case 'L':
                rotate_about_fl(motor_pwm);
                break;
            case 'R':
                rotate_about_fr(motor_pwm);
                break;
            default:
                ERROR_FLAG = true;
                break;
            }
            break;
        case 'B':
            //Left/right
            switch(cmd_direction[1]){
            case 'L':
                rotate_about_bl(motor_pwm);
                break;
            case 'R':
                rotate_about_br(motor_pwm);
                break;
            default:
                ERROR_FLAG = true;
                break;
            }
            break;
        default:
            ERROR_FLAG = true;
            break;
        }
        break;

    //Translate CMDs
    case 'T':
        //Left/right
        switch(cmd_direction[0]){
        case 'L':
            translate_left(motor_pwm);
            break;
        case 'R':
            translate_right(motor_pwm);
            break;
        default:
            ERROR_FLAG = true;
            break;
        }
        break;

    //Diagonal CMDs
    case 'D':
        //Front/back
        switch(cmd_direction[0]) {
        case 'F':
            //Left/right
            switch(cmd_direction[1]){
            case 'L':
                diagonal_over_fl(motor_pwm);
                break;
            case 'R':
                diagonal_over_fr(motor_pwm);
                break;
            default:
                ERROR_FLAG = true;
                break;
            }
            break;
        case 'B':
            //Left/right
            switch(cmd_direction[1]){
            case 'L':
                diagonal_over_bl(motor_pwm);
                break;
            case 'R':
                diagonal_over_br(motor_pwm);
                break;
            default:
                ERROR_FLAG = true;
                break;
            }
            break;
        default:
            ERROR_FLAG = true;
            break;
        }
        break;

    //Program CMD
    case 'P':
        brake_all();
        motor_pwm = input_pwm&0xff;
        break;

    //Error Reset CMD
    case 'E':
        ERROR_FLAG = false;
        break;

    //BAD CMD!
    default:
        ERROR_FLAG = true;
    }

    if(ERROR_FLAG) {
        strcpy(error_buffer,"!BADCMD");
    }
}

void demo_action() {
    for(int i=0; i<4; i++){
        drive_forward(100);
        delay(500);
        brake_all();
        delay(500);
        drive_backward(100);
        delay(500);
        brake_all();
        delay(500);
    }
}

void drive_forward(int pwm) {
    set_motors(pwm,pwm,pwm,pwm);
}

void drive_backward(int pwm) {
    set_motors(-pwm,-pwm,-pwm,-pwm);
}

void brake_all() {
  brake(fl_motor,fr_motor);
  brake(bl_motor,br_motor);
}

void translate_left(int pwm) {
    set_motors(-pwm,pwm,pwm,-pwm);
}

void translate_right(int pwm) {
    set_motors(pwm,-pwm,-pwm,pwm);
}

void diagonal_over_fl(int pwm) {
    set_motors(0,pwm,pwm,0);
}

void diagonal_over_fr(int pwm) {
    set_motors(pwm,0,0,pwm);
}

void diagonal_over_bl(int pwm) {
    set_motors(-pwm,0,0,-pwm);
}

void diagonal_over_br(int pwm) {
    set_motors(0,-pwm,-pwm,0);
}

void rotate_left(int pwm) {
    set_motors(-pwm,pwm,-pwm,pwm);
}

void rotate_right(int pwm) {
    set_motors(pwm,-pwm,pwm,-pwm);
}

void rotate_about_fl(int pwm) {
    set_motors(0,-pwm,0,-pwm);
}

void rotate_about_fr(int pwm) {
    set_motors(-pwm,0,-pwm,0);
}

void rotate_about_bl(int pwm) {
    set_motors(0,pwm,0,pwm);
}

void rotate_about_br(int pwm) {
    set_motors(pwm,0,pwm,0);
}

void set_motors(int fl_pwm, int fr_pwm, int bl_pwm, int br_pwm) {
    if(fl_pwm == 0) {
        fl_motor.brake();
    } else {
        fl_motor.drive(fl_pwm);
    }
    if(fr_pwm == 0) {
        fr_motor.brake();
    } else {
        fr_motor.drive(fr_pwm);
    }
    if(bl_pwm == 0) {
        bl_motor.brake();
    } else {
        bl_motor.drive(bl_pwm);
    }
    if(br_pwm == 0) {
        br_motor.brake();
    } else {
        br_motor.drive(br_pwm);
    }
}
