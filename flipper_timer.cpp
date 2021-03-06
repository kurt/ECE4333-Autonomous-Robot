#include "mbed.h"
 
Ticker flipper;
Ticker Interupt;
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

void flip() {
    led2 = !led2;
}

void flipthree(){
    led3=!led3;
 }
 
int main() {
    led2 = 1;
    flipper.attach(&flip, 2.0); // the address of the function to be attached (flip) and the interval (2 seconds)
    Interupt.attach(&flipthree,0.2);
    // spin in a main loop. flipper will interrupt it to call flip
    while(1) {
        led1 = !led1;
        wait(0.2);
    }
}