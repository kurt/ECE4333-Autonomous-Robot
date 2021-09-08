/*
script: lab_3.c
authors: Kurt Stewart
         James McMurtie
         
Adapted from code provided by Dr. Chris Diduch UNB

description: develops a PI control thread to control the speed of a 
DC motor configured through SPI interfacing. Periodic threading is used
to ensure the periodic sampling and setting of the output speed within
the closed loop controller.

motor 1: left side of robot
motor 2: right side of robot
*/

#include "rtos.h"
#include "mbed.h"
#include <math.h>

// Function prototypes
void ReadSPIvel();
void PiControllerISR(void); //void PiControllerISR(void const *n);
void FeedbackISR(void);
void WatchdogISR(void const *n);
void WatchdogThread(void const *argument);
void PiControllerThread(void const *argument);
void SetSPI(SPI DEO);
float SaturatingSubtract(float x, float y);
float SaturateValue(float x, float Limit);
float SaturatingAddition(float x,float y);
void FeedbackThread(void const *argument);
void SetSPI(void);
float RangeFind(unsigned tof);
int Obstruction_Detect(unsigned range);
void Angle_Update(void);
void move_to_other_wall(void);


// Processes and threads
int32_t SignalWatchdog;
int32_t SignalPiController;
int32_t SignalFeedback;
osThreadId WatchdogId, ExtInterruptId, PeriodicInterruptId; 
osThreadId PiControllerId;
osThreadId FeedbackId;
osTimerDef(Wdtimer, WatchdogISR); // Declare a watch dog timer

// Declare WatchdogThread as a thread/process
osThreadDef(WatchdogThread, osPriorityRealtime, DEFAULT_STACK_SIZE); 

// Declare PiController thread as a thread
osThreadDef(PiControllerThread, osPriorityHigh, DEFAULT_STACK_SIZE); 
osThreadDef(FeedbackThread, osPriorityNormal, DEFAULT_STACK_SIZE); // 


// IO Port Configuration
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
PwmOut PwmP22(p22),PwmP21(p21);
DigitalOut dir(p9),dir2(p10);
DigitalOut brake(p19), brake2(p20);



// Reset for all devices within the slave SPI // peripheral in the DE0 FPGA
// Places SPI interface (within the DE0 FPGA) into control mode
 DigitalOut IoReset(p15);  
 
 // Defines DE0 as a SPI channel connected to the FPGA that implements the QEI 
 DigitalOut SpiReset(p14); 
 SPI DE0(p5, p6, p7); // mosi, miso, sclk
 
 
Serial pc(USBTX, USBRX); // Pins (tx, rx) for PC serial channel
InterruptIn Bumper(p8); // External interrupt pin declared as Bumper
Ticker PeriodicInt; // Declare a timer interrupt: PeriodicInt- for PI control
Ticker PeriodicFBInt; //Used for feedback loop

// Declare global variables
int Position;
int sign, t, duty; //int PWMval;
float x=0,x2=0,T,y,ytwo,uProportional,uProportional2, uIntegral,uIntegral2;
float u=0,u2=0, PWMval,PWMval2;
float r,e=0,e2=0, dTime,dTime2, id ,elimit=99,xLimit=0xFFFF, xTemp, xTemp2;
float w,w2,Vel,Vel2,N= 16;
float set_point=0,set_point2=0,steerL,steerR;
int Corner_Flag=0;
float L=0.2159; //meters
float rw=0.508; //meters
float theta=0;  // degrees
int dPosition,dPosition2; 
float Pos=0, Pos2=0;


// ********************** Main Thread *****************************************
int main() { // This thread executes first upon reset or power-on.
brake=1;
brake2=1;

PiControllerId = osThreadCreate(osThread(PiControllerThread), NULL);
FeedbackId=osThreadCreate(osThread(FeedbackThread),NULL);
//signed int u;
PeriodicInt.attach(&PiControllerISR, 0.001);//0.05

//need to do this less frequently than SPI and trggering/echo time.
PeriodicFBInt.attach(&FeedbackISR,0.05);

T=0.001;  // Period in Seconds
y=0;      // Duty Cycle

int16_t id;
id=0x1;
PwmP22.period(T);
//PwmP22.write(y);
PwmP21.period(T);
//PwmP21.write(y);

//osTimerId Interruptsdsokd = osTimerCreate(osTimer(RandomName), osTimerPeriodic, (void *)0);
//osTimerStart (Interruptsdsokd,2);       // time of interrupt-> 2 ms

// Start the watch dog timer and enable the watch dog interrupt
//osTimerId OneShot = osTimerCreate(osTimer(PiControl), osTimerOnce, (void *)0);
//osTimerStart(OneShot, 2); // Start the  timer interrupt and set to 2ms.


pc.printf("\r\n Hello World - RTOS Template Program\n\r");

SetSPI();

id=DE0.write(0x8002); // Specify a read only transactions of 2 words 8402 8002
 pc.printf("%d\r\n", id); 
    if (id==0x17){ // Checks for good communication
        pc.printf("Good Communication\n\r");}
    else{pc.printf("Bad Communication\n\r");}

set_point=0.02;
set_point2=0.0101;
 
//Loop in main
do {
 pc.printf("Error = %f\r\n",e);
 //if (pc.readable()){
 //pc.scanf("%f",u);
 //x=pc.getc();
 //pc.putc(x); //Echo keyboard entry
 //}
 led4=!led4;   
    pc.printf("Vel1 is : %f\r\n Vel2 is : %f\r\n\n\n",Vel,Vel2);
    //pc.printf("", u);
    //pc.printf("", y);
    //pc.printf("", PWMval);
    //pc.printf("", x);
    //pc.printf("", dPosition);

//pc.printf("\r\n Hello World - RTOS Template Program");
}
while(1);
//end of main loop
}
// *************************end of MAIN!!!!!!! ********************************


// ******** Watchdog Thread ********
void WatchdogThread(void const *argument) {
while (true) {
 // Go to sleep until a signal, SignalWatchdog, is received
 osSignalWait(SignalWatchdog, osWaitForever); 
 led1 = ~led1;
 }
}


// ******** Watchdog Interrupt Handler ********
void WatchdogISR(void const *n) {
 // Send signal to thread with ID, WatchdogId, i.e., WatchdogThread.
 osSignalSet(WatchdogId,0x1); 
}



/*
// ******** External Interrupt Thread ********
void ExtInterruptThread(void const *argument) {
while (true) {
 // Go to sleep until signal, SignalExtCollision, is received
 osSignalWait(SignalExtInterrupt, osWaitForever); 
 led2 = !led2;
 }
}

*/

/*
// ******** External Interrupt Handler ********
void ExtInterruptISR(void) {
 // Send a signal to thread with ID, ExtInterrupt, i.e., ExtInterruptThread.
 osSignalSet(ExtInterruptId,0x2); 
}
*/


// ******** PI Control Tread ****************
void PiControllerThread(void const *argument) {

float uS,uS2,Kp=5, Ki=0.003, uLimit=0.5,i=0.000001,rmax;//0.5
    
    rmax= 0.05;//0.05 0.008
    
    
    
    while (1){
        osSignalWait(SignalPiController, osWaitForever);
        ReadSPIvel();
        led3=!led3; //toggles LED everytime the thread is called
        //r+=i;   // Creates Ramp input
        
        //limit set_points to rmax
        if(set_point<=rmax&&set_point>=0){set_point=set_point;}
        else{set_point=rmax;}
        if(set_point2<=rmax&&set_point2>=0){set_point2=set_point2;}
        else{set_point2=rmax;}
        //r=0.05;
        w=Vel;
        w2=Vel2;
        e=SaturatingSubtract(set_point,w); 
        e2=SaturatingAddition(set_point2,w2);
        e=SaturateValue(e,elimit);
        e2=SaturateValue(e2,elimit);
        //e=r-w;
        
        xTemp=SaturatingAddition(x,e);
        xTemp2=SaturatingAddition(x2,e2);
        xTemp=SaturateValue(xTemp, xLimit);
        xTemp2=SaturateValue(xTemp2,xLimit);
        //xTemp=x+e;
        
        //Scaled proportional an  d integral control terms
        uProportional=(Kp*e);
        uProportional2=(Kp*e2);
        uIntegral=(Ki*xTemp);
        uIntegral2=(Ki*xTemp2);
        uS=SaturatingAddition(uProportional,uIntegral);
        uS2=SaturatingAddition(uProportional2,uIntegral2);
        //uS=uProportional+uIntegral;
        //uS2=uProportional2+uIntegral2;
        
        u=SaturateValue(uS, uLimit);
        u2=SaturateValue(uS2,uLimit);
        if(u==uS) {x=xTemp;} // integrator windup prevention
        if(u2==uS2){x2=xTemp2;}// integrator windup prevention
        
        if(u>=0){ dir=1 ;} else {dir=0;}//>=
        if (u2>=0){dir2=0;} else {dir2=1;}// 
        PWMval=(abs(u));
        PWMval2=(abs(u2));
        y=PWMval;        // val of duty left motor
        ytwo=PWMval2;      // right motor
        //pc.printf(" Duty cycle float = %f\r\n\n\n",y);
        
        PwmP22.write(y);//left motor
        PwmP21.write(ytwo);//right motor
        //brake=0;
        //brake2=0;
    }
    //end of while
    //wait(0.1); // Go to sleep for 500 ms
}// end of PIControlThread

// ******** PiConttroller Timer Interrupt Handler ********
void PiControllerISR(void) {
 // Activate the signal, PiController, with each periodic timer
 osSignalSet(PiControllerId,0x1); 
 // interrupt.
 }

 
 
 // ******** Set Motor Parameters ********

 
 // end Set Motor Parameters
 
 //
 
void IOReset(){
    IoReset=0;
    IoReset=1;
    IoReset=0; 
}

 
 // ******** Set SPI ********
 
 void SetSPI(void){
    DE0.format(16,1);       // SPI format: 16-bit words, mode 1
    DE0.frequency(1000000); // Set SPI bit rate (Hz). Default is 1 MHz
    IoReset=0;
    IoReset=1;
    wait_us(5);
    IoReset=0; 

    SpiReset = 0; // Restart with a new SPI protocol.
    SpiReset = 1; //
    wait_us(5);
    SpiReset = 0; // Restart with a new SPI protocol.

    SpiReset = 0; // Restart with a new SPI protocol.
    SpiReset = 1; //
    wait_us(5);
    SpiReset = 0; // Restart with a new SPI protocol.
    
}
// end Set SPI

 
// *************** Read SPI *****************
void ReadSPIvel(){
    int16_t V;
    SpiReset=0;
    SpiReset=1;
    wait_us(1);
    SpiReset=0;
    V=DE0.write(0x8004);
    dPosition = DE0.write(0x1);           // Read QEI-0 position register
    dTime=DE0.write(0x1);                 // Read QEI-0 time register
    dPosition2=DE0.write(0x1);            // Read QEI-1 position register 
    dTime2=DE0.write(0x1);                // Read QEI-1 time register
    // this doesnt seem to be helping
    if (dPosition & 0x00008000) {dPosition = dPosition | 0xFFFF0000;}
        // Units of dP are in ‘counts’
        // 1 count represents 2π/(4*N) rad
    if (dPosition2 & 0x00008000) {dPosition2 = dPosition2 | 0xFFFF0000;}
    // Units of dP are in ‘counts’
    // 1 count represents 2π/(4*N) rad
    Pos=Pos+dPosition;                    // units of counts
    Pos2=Pos2+dPosition2;                 // units of counts
    //Vel = 9766*3.14*dPosition/(dTime*N);// units of rad/s
    Vel = dPosition/dTime;
    Vel2= dPosition2/dTime2;
}
// **********  END Read SPI ********************
 
// Saturating Subtract: takes the difference between two inputs and 
// saturates the negative value 
float SaturatingSubtract(float x, float y){
    float z; 
    z =  x - y; // 32-bit overflow detection and saturating arithmetic 
    if((x > 0) && (y < 0) && (z < 0)) z = 0x7FFFFFFF; 
    else if((x < 0) && (y > 0) && (z > 0)) z = 0x80000000; 
    return z;
}
// end of Saturating Subtract

// Saturating addition: Addition of two inputs and saturates the positive value 
 float SaturatingAddition(float x, float y){
    float z; 
    z =  x + y; // 32-bit overflow detection and saturating arithmetic 
    if((x > 0) && (y > 0) && (z < 0)) z = 0x7FFFFFFF;  
    else if((x<0) && (y<0) && (z>0)) z = 0x80000000;
    return z;
}

// Saturate Value: ensures that the maximum value that is possibly 
// outputted does not exceed the maximum value allowable
float SaturateValue(float x, float Limit){
    if(x > Limit) return(Limit); 
    // Impose maximum limit on x 
    else if(x < -Limit) return(-Limit); 
    else return(x);
 }
 
 // ******** Feedback Timer Interrupt Handler ********
void FeedbackISR(void) {
 // Activate the signal, FeedbackID, with each periodic timer
 osSignalSet(FeedbackId,0x1); 
 // interrupt.
 }
 
 // Get the distance and returns the setpoint obstruction matrix
void FeedbackThread(void const *argument){
 while(1){
    osSignalWait(SignalFeedback, osWaitForever);
    led2=!led2;
    unsigned Dist_Matrix[11]={},Obs_Matrix[11]={},V;
    float control[2]={};
    int i,count, dummy=0x1,count_a;
    SpiReset=0;
    SpiReset=1;
    wait_us(5);
    SpiReset=0;
    V=DE0.write(0x840C); // Control word to read 12 ultrasonic range finders
    for (i=0; i<12; i++) {
            // Read 16-bit unsigned time of flight in us.
            Dist_Matrix[i]=DE0.write(dummy); 
            Dist_Matrix[i]=RangeFind(Dist_Matrix[i]);
            Obs_Matrix[i]=Obstruction_Detect(Dist_Matrix[i]);
            //stop the robot if there is an issue (obstacle in front
     }//end of for get data from sensors    
            if (Obs_Matrix[0]==1||Obs_Matrix[11]==1||Obs_Matrix[1]==1)
                {brake=1,brake2=1;}
            else if (Dist_Matrix[3]>30 || Dist_Matrix[4]>30){
                set_point=set_point2+0.001;
                brake=0;
                brake2=0;
            }
            else {set_point=0.01,set_point2=0.012;}
            //if ((Dist_Matrix[0]<=15)||Corner_Flag){move_to_other_wall();}
    

        }//end of while
}//end of FeedbackThread
 
 
 /* 
 void Determine_Direction(unsigned *Obs_Matrix){
    //changes direction if needed according to environment sensors
    
    int front_ind=0; 
if ((!Obs_Matrix(front_ind))&&(Obs_Matrix(2))&&(Obs_Matrix(3))&&(!Corner_Flag)){
        //continue along wall case
        set_point=set_point;
        set_point2=set_point2;
        }
    else if (Obs_Matrix(front_ind)){
        //obstruction in the way: wall in front
        move_to_other_wall();// needs to set corner Flag to 
                             // zero when this is done
        Corner_Flag=1;     
     }
     else{
        follow_wall();   
     }   
}//end of Determine_Direction
*/
/*

void follow_wall(void){
    // minimize error - set to a certain distance away from the wall
       while (wall_follower)
    return
    }
*/
void move_to_other_wall(void){
    // turn in the direction opposite to present wall until at 
    // 90 degrees from current position
    while (theta<90){ //maybe 2*3.14;
        //turn the robot by changing the setpoints
        //steerL+=0.001;
        steerR+=0.01;
        //set_point=0.0001;
        //set_point2=0.0001;
        Angle_Update();
        led1=!led1;
        set_point2=set_point2+steerR;
        } 
    Corner_Flag=0;
    theta=0;
    return;
}


//________________Range Find____________________________    
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

//________________Obstruction Detect______________________
int Obstruction_Detect(unsigned range){
     //identifies if there is an object that is too close 
     // (6 cm) and sets a logic state 
     unsigned too_close=15; //cm
     int obj_trig;      //a boolean that states if in trouble or not
     if (range<=too_close){obj_trig=1;}
     else obj_trig=0;
     return obj_trig;
}
//end of Obstruction_Detect

void Angle_Update(void){
    float deltatheta;
    deltatheta=rw/L*w2-rw/L*w; //
    theta+=deltatheta;
    }