#include <Wire.h>

// variable for reading input
char input_str[100];
int input_num = 0;
int input_byte = 0;
int input_sep = 0;
char tmp[100];

// setup function should declare first
void armSetup();
void carSetup();

void setup() {
    Serial.begin(115200);
    myPrint("sys", "Start");

    carSetup();
    armSetup();
}


void loop() {
    readInput();
    // and do nothing
    // delay(1);
}


// A log function
template<class T>
void myPrint(char* log_type, T c) {
    Serial.print('[');
    Serial.print(log_type);
    Serial.print("] ");
    Serial.println(c);
}


// Read serial input
void readInput() {
    if (Serial.available() > 0) {
        // read one char
        input_byte = Serial.read();
        input_str[input_num] = input_byte;
        // myPrint("debug", input_str[input_num]);

        // text
        if (isGraph(input_str[input_num])) {
            input_str[++input_num] = '\0';
        } else
            return;

        // stop
        if(input_str[input_num - 1] == ';') {
            run_command();
            // echo when fininshed
            myPrint("echo", input_str);
            input_num = input_sep = 0;
            return;
        }

        // seprate char
        if(input_str[input_num - 1] == ',') {
            input_sep = input_num - 1;
        }
    }
}


// Run the input command after reading from serial
void run_command() {
    // wheel: c{mode}{speed},{displacement}
    // e.g. `cf300,30;` `cb100,40;`
    if (input_sep > 0 && input_str[0] == 'c') {
        char mode = input_str[1];
        input_str[input_sep] = '\0';
        int sped = atoi(input_str + 2),
            dist = atoi(input_str + (input_sep + 1));
        sprintf(tmp, "Car mode %c move %d with speed %d", mode, dist, sped);
        myPrint("car", tmp);
        carMove(dist, sped, mode);
        input_str[input_sep] = ',';
    }

    // arm: a{device},{angle};
    // e.g. `a0,120;` `a1,150;`
    if (input_sep > 0 && input_str[0] == 'a') {
        int dev = input_str[1] - '0';
        int ang = atoi(input_str + (input_sep + 1));
        // sprintf(tmp, "Arm %d move to %d", dev, ang);
        // myPrint("arm", tmp);
        armMove(dev, ang);
    }

    // reset
    // e.g. r0, r1
    if (input_num >= 2 && input_str[0] == 'r' && input_str[1] == '0') {
        armReset();
        return;
    }
}
