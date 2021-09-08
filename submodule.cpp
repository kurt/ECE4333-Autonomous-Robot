#include "rtos.h"
#include "mbed.h"
#include "Timer.h"
#include <math.h>
 
float RangeFind(unsigned tof);
int Obstruction_Detect(unsigned range);
//void Determine_Direction(unsigned *Obs_Matrix);
//void follow_wall(void);

//unsigned int UsSPIWrite();
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
DigitalOut IoReset(p15); // Places SPI interface (within the DE0 FPGA) // into control mode 
DigitalOut SpiReset(p14); // Defines DE0 as a SPI channel connected to // the FPGA that implements the QEI 
DigitalOut US_Trig(p27);// trigger us 
InterruptIn US_Count(p28);// count us
SPI DE0(p5, p6, p7);
Serial pc(USBTX, USBRX); // Pins (tx, rx) for PC serial channel
int Corner_Flag=0;

int main(){
    pc.printf("\r\n Hello World - RTOS Template Program\n\r");
    //us_read=UsSPIWrite();
    unsigned V;
    unsigned Dist_Matrix[12]={},Obs_Matrix[12]={};
    int i,count, dummy=0x1,count_a; //Dummy may need to be changed from an int type
    int16_t id;
    
    DE0.format(16,1); // SPI format: 16-bit words, mode 1
    DE0.frequency(1000000); // Set SPI bit rate (Hz). Default is 1 MHz
    IoReset=0;
    IoReset=1;
    wait_us(5);
    IoReset=0;
    
    SpiReset=0;
    SpiReset=1;
    wait_us(5);
    SpiReset=0;
    
    SpiReset=0;
    SpiReset=1;
    wait_us(5);
    SpiReset=0;
    
    id=0x1;
    //do{
    id=DE0.write(0x0002); // Specify a read only transactions of 2 words 8402 8002
    pc.printf("ID : %d\r\n", id); 
    
    if (id==0x17){ // Checks for good communication
        pc.printf("Good Communication\n\r");}// Good Communication
    /*else{ // Bad Communication
        led1=1;
        led2=1;
        led3=1;
        led4=1;
        id=0x1;
        pc.printf("Bad Communication\r\n\n");
        wait_ms(500);
        led1=0;
        led2=0;
        led3=0;
        led4=0;
        SpiReset=0;
        SpiReset=1;
        wait_us(5);
        SpiReset=0;
        wait_ms(500);
       }
       }
       while(id==0x1);
      */
        //led2
        //V=DE0.write(0x8401); // Control word to read 12 ultrasonic range finders
        do{
        SpiReset=0;
        SpiReset=1;
        wait_us(5);
        SpiReset=0;
        V=DE0.write(0x840C); // Control word to read 12 ultrasonic range finders
         //dont need to do this as DE0 board does triggering
       
        
        //if US_Count.rise(
        //needs to be divided up into the three phases
        //W=DE0.write(dummy);
        //Get US Data and determine if there is an obstacle
        for (i=0; i<12; i++) {
            Dist_Matrix[i]=DE0.write(dummy); // Read 16-bit unsigned time of flight in us.
            Dist_Matrix[i]=RangeFind(Dist_Matrix[i]);
            Obs_Matrix[i]=Obstruction_Detect(Dist_Matrix[i]);
            //pc.printf("OBS : %d\r\n", Obs_Matrix[i]);
            pc.printf("OBS : %d\r\n", Dist_Matrix[i]);
        }//end of get data from sensors
        count_a=Dist_Matrix[0];
        
        //Determine_Direction(Obs_Matrix);
        
        //pc.printf("Count Value %d\n\r",W); //
        //printf("Count so far: %d\n", counter.read());
        led1=!led1; 
        pc.printf("\r\n Hello World - RTOS Template Program\n\r");
        wait_us(1e3);// wait for next trigger ping there is a 50 ms distance between each trigger per phase
        US_Trig=0;
        wait_us(2);
        US_Trig=1;
        wait_us(100); 
        US_Trig=0;
        }while(1);

}
//end of main!!!
  
//*************************** Functions and Threads ************************************************
    
 /*   
 //Read from all the SPI write
 *unsigned UsSPIWrite(){   
    unsigned V, W;
    SpiReset=0;
    SpiReset=1;
    wait_us(5);
    SpiReset=0;
    V=DE0.write(0x840C); // Control word to read 12 ultrasonic range finders
    for (i=1; i<12; i++) {
    W(i)=DE0.write(Dummy); // Read 16-bit unsigned time of flight in us.
    }
    return *W;
}


// ******** External Interrupt Thread ********
void ExtInterruptThread(void const *argument) {
while (true) {
 osSignalWait(SignalExtInterrupt, osWaitForever); // Go to sleep until signal, SignalExtCollision, is received
 led2 = !led2;
 }
}

// ******** External Interrupt Handler ********
void ExtInterruptISR(void) {
 osSignalSet(ExtInterruptId,0x1); // Send a signal to thread with ID, ExtInterrupt, i.e., ExtInterruptThread.
}

// ******** Periodic Timer Interrupt Thread ********
void PeriodicInterruptThread(void const *argument) {
while (true) {
 osSignalWait(SignalPeriodicInterrupt, osWaitForever); // Go to sleep until signal, SignalPi, is received.
 led3= !led3; // Alive status - led3 toggles each time PieriodicZInterruptsThread is signaled.
 Position = Position + 1;
 }
}
// ******** Period Timer Interrupt Handler ********
void PeriodicInterruptISR(void) {
 osSignalSet(PeriodicInterruptId,0x1); // Send signal to the thread with ID, PeriodicInterruptId.
 }
 */
 
  float RangeFind(unsigned tof){
     // Returns range in cm of an ultrasonic range finder
     float m,y,R,T,v,range;
     m=0.0288;
     y=1.4;
     R= 8.314;
     T= 20;                             // temperature in celcius
     v=sqrt((y*R*(T+273))/m);           //velocity of sound
     range=v*(float)tof/2;
     range=range/10000;
     return range;
}
//end of RangeFind 
     
  int Obstruction_Detect(unsigned range){
     //identifies if there is an object that is too close (6 cm) and sets a logic state 
     // if the 
     unsigned too_close=6; //cm
     int obj_trig;      //a boolean that states if in trouble or not
     if (range<=too_close){obj_trig=1;}
     else obj_trig=0;
     return obj_trig;
}
//end of Obstruction_Detect
     
/*
 void Determine_Direction(unsigned *Obs_Matrix){
    //changes direction if needed according to environment sensors
    
    int front_ind=0, 
    if (!Obs_Matrix(front_ind)&&Obs_Matrix(2)&&Obs_Matrix(3)&&!Corner_Flag){
        //continue along wall case
        set_point=set_point;
        }
    else if (Obs_Matrix(front_ind)){
        //obstruction in the way: wall in front
        move_to_other_wall();//needs to set corner Flag to zero when this is done
        Corner_Flag=1;     
     }
     else{
        follow_wall();   
     }
     return;    
}//end of Determine_Direction



void follow_wall(void){
    // minimize error - set to a certain distance away from the wall
       while (wall_follower)
    return
    }

void move_to_other_wall(void){
    // turn in the direction opposite to present wall until at 90 degrees from current position
    while (angle<90){
        //turn the robot by changing the setpoints
        
        } 
    Corner_Flag=0;
    return;
    }
    */