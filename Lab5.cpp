#include "mbed.h"

PwmOut PWM_out(p21);
DigitalOut led3(LED3); 
DigitalOut led4(LED4); 
AnalogIn A_0(p20);
AnalogIn A_1(p19);

int main() {
    // specify period first
    float Amotor;
    while (1) {
    PWM_out.period(4.0e-3);      // 4e-3 second period
    PWM_out.write(0.80f);      // 50% duty cycle, relative to period
    if (A_1>0.2f){ 
        //run session
        led3=!led3;
        led4=0;
        Amotor=A_0; //Amotor=A_0/0.179;
        printf("%f\r\n", Amotor);
        if (Amotor>0.37){
            //DigitalOut cut(p20)
            PWM_out.write(0);
            break;
        }   
    }else{
        //stop session
        led4=1;
        PWM_out.write(0);
    }
    //printf("percentage: %3.3f%%\n", A_1.read()*100.0f);
    
    //led = 0.5f;          // shorthand for led.write()
    //led.pulsewidth(2);   // alternative to led.write, set duty cycle time in seconds
    }
    //while(1);
}