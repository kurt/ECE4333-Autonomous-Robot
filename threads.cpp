#include "mbed.h"
#include "rtos.h"
 
DigitalOut led1(LED1);
DigitalOut led2(LED2);
Thread thread;
 
void led2_thread() {
    while (true) {
        led2 = !led2;
    }
}
 
int main() {
    thread.start(led2_thread);
    
    while (true) {
        led1 = !led1;
        wait_ms(500);
    }
}