/**
 * Simple example of how to use the commander without serial - using just strings
 */

#include <SimpleFOC.h>

// instantiate the commander
Commander command = Commander();

// led control function
void doLed(char* cmd){ 
    if(atoi(cmd)) gpio_put(LED_BUILTIN, HIGH); 
    else gpio_put(LED_BUILTIN, LOW); 
};
// get analog input 
void doAnalog(char* cmd){ 
    if (cmd[0] == '0') Serial.println(analogRead(A0));
    else if (cmd[0] == '1') Serial.println(analogRead(A1));
};

void setup() {
    // define pins
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);

    // Serial port to be used
    Serial.begin(115200);

    // add new commands
    command.add('L', doLed, "led control");
    command.add('A', doAnalog, "analog read A0-A1");

    printf("Commander running\n");
    sleep_ms(1000);
}


void loop() {
    // user communication
    command.run("?"); 
    sleep_ms(2000);
    command.run("L0"); 
    sleep_ms(1000);
    command.run("A0"); 
    sleep_ms(1000);
    command.run("A1"); 
    sleep_ms(1000);
    command.run("L1"); 
    sleep_ms(s(1000);
}