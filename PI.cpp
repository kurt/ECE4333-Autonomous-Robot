#include "mbed.h"
#include "rtos.h"

void PiControllerISR(void); //void PiControllerISR(void const *n);
void FeedbackISR(void);
void PiControllerThread(void const *argument);
void FeedbackThread(void const *argument);

float SaturatingSubtract(float x, float y);
float SaturateValue(float x, float Limit);
float SaturatingAddition(float x,float y);

void SetSPI(SPI DEO);


int32_t SignalPiController;
int32_t SignalFeedback;

osThreadId PiControllerId;
osThreadId FeedbackId;

osThreadDef(PiControllerThread, osPriorityNormal, DEFAULT_STACK_SIZE); // Decalre PiController thread as a thread
osThreadDef(FeedbackThread, osPriorityNormal, DEFAULT_STACK_SIZE);


DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
PwmOut PwmP22(p22);
DigitalOut dir(p23);
DigitalOut brake(p24);

// Reset for all devices within the slave SPI // peripheral in the DE0 FPGA
 DigitalOut IoReset(p15); // Places SPI interface (within the DE0 FPGA) // into control mode 
 DigitalOut SpiReset(p14); // Defines DE0 as a SPI channel connected to // the FPGA that implements the QEI 
 SPI DE0(p5, p6, p7);

Serial pc(USBTX, USBRX); // Pins (tx, rx) for PC serial channel
Ticker PeriodicInt; // Declare a timer interrupt: PeriodicInt


//--------------------START OF MAIN--------------------------//
int main() {
    
    DE0.format(16,1); // SPI format: 16-bit words, mode 1
    DE0.frequency(1000000); // Set SPI bit rate (Hz). Default is 1 MHz
    IoReset=0;
    IoReset=1;
    IoReset=0; 

    SpiReset = 0; // Restart with a new SPI protocol.
    SpiReset = 1; //
    wait_us(10);
    SpiReset = 0; // Restart with a new SPI protocol.
    
    PiControllerId = osThreadCreate(osThread(PiControllerThread), NULL);
    FeedbackId=osThreadCreate(osThread(FeedbackThread),NULL);
    PeriodicInt.attach(&PiControllerISR, 0.001);
    //PeriodicInt.attach(&FeedbackISR, 0.001);
    
    led1=0;
    led2=0;
do{

    //PiControllerId = osThreadCreate(osThread(PiControllerThread), NULL);
    //PeriodicInt.attach(&PiControllerISR, 1);

    pc.printf("\r\n Hello World - RTOS Template Program\n\r");

    led1=!led1;

}
while(1);
}

// --------------------END OF MAIN ------------------------//



/*
 // ******** Set SPI ********
 void SetSPI(DE0){
    DE0.format(16,1); // SPI format: 16-bit words, mode 1
    DE0.frequency(1000000); // Set SPI bit rate (Hz). Default is 1 MHz
    IoReset=0;
    IoReset=1;
    IoReset=0; 

    SpiReset = 0; // Restart with a new SPI protocol.
    SpiReset = 1; //
    wait_us(10);
    SpiReset = 0; // Restart with a new SPI protocol.
    
}
// end Set SPI
 */


// ******** PI Control Tread ****************
void PiControllerThread(void const *argument) {
    while (1){
        osSignalWait(SignalPiController, osWaitForever);
        led2=!led2;
        }
        /*
        w=Vel;
        e=SaturatingSubtract(r,w); 
        etemp=e;
        e=SaturateValue(e,elimit);
        xTemp=SaturatingAddition(x,e);
        xTemp=SaturateValue(xTemp, xLimit);
        //Scaled proportional and integral control terms
        uProportional=(Kp*e);
        uIntegral=(Ki*xTemp);
        uS=SaturatingAddition(uProportional,uIntegral);
        u=SaturateValue(uS, uLimit);
        if(u=uS) {x=xTemp;} /// integrator windup prevention
        if(u>=0) dir=1; else dir=0;
        PWMval=(abs(u));
        y=T*PWMval;        // val of duty   
        PwmP22.write(y);
        brake=0;
        */
    
}
//end of PiControllerThread

// ******** PiConttroller Timer Interrupt Handler ********
void PiControllerISR(void) {
 osSignalSet(PiControllerId,0x1); // Activate the signal, PiController, with each periodic timer
 // interrupt.
 }
 //End of PI ISR
 
void FeedbackThread(void const *argument) {
 while(1){
    led4=!led4;
    }
}


// ******** PiConttroller Timer Interrupt Handler ********
void FeedbackISR(void) {
 osSignalSet(FeedbackId,0x1); // Activate the signal, PiController, with each periodic timer
 // interrupt.
 }



 
 
 
 
 /// PI Calculations
  // Saturating Subtract: takes the difference between two inputs and saturates the negative value 
 float SaturatingSubtract(float x, float y){
    float z; 
    z =  x - y; // 32-bit overflow detection and saturating arithmetic 
    if((x > 0) && (y < 0) && (z < 0)) z = 0x7FFFFFFF; 
    else if((x < 0) && (y > 0) && (z > 0)) z = 0x80000000; 
    return z;
}

// Saturating addition: takes the difference between two inputs and saturates the negative value 
 float SaturatingAddition(float x, float y){
    float z; 
    z =  x + y; // 32-bit overflow detection and saturating arithmetic 
    if((x > 0) && (y > 0) && (z < 0)) z = 0x7FFFFFFF;  
    return z;
}

// Saturat Value: ensures that the maximum value that is possibly outputted does not exceed the maximum value allowable
float SaturateValue(float x, float Limit){
    if(x >= Limit) return(Limit); 
    // Impose maximum limit on x 
    else if(x < -Limit) return(-Limit); 
    else return(x);
 }
 
