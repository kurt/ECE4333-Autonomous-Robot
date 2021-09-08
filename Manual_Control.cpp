/*
script: Manual Control
authors: Kurt Stewart
         James McMurtie
         
Adapted from Lab3 code, and code provided by Dr. Diduch for UNB course ECE 4333

description:    Manual control development,
                PI controller adjustment for driving backwards

motor 1: left side of robot
motor 2: right side of robot
*/

#include "rtos.h"
#include "mbed.h"
#include <math.h>

// Function prototypes
void ReadSPIvel();
void FeedbackISR(void);
void PiControllerISR(void); //void PiControllerISR(void const *n);
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
int32_t SignalPiController;
int32_t SignalFeedback;
osThreadId ExtInterruptId, PeriodicInterruptId; 
osThreadId PiControllerId;
osThreadId FeedbackId;

// Declare PiController thread as a thread
osThreadDef(PiControllerThread, osPriorityHigh, DEFAULT_STACK_SIZE);

// Declare Feedbackthread thread as a thread
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
 DigitalOut IoReset(p15); // Places SPI interface (within the DE0 FPGA)
 DigitalOut SpiReset(p16); // Defines DE0 as a SPI channel
 SPI DE0(p5, p6, p7); // mosi, miso, sclk
 
 
Serial pc(USBTX, USBRX); // Pins (tx, rx) for PC serial channel
Serial Bt(p13, p14); // tx, rx; Bluetooth communication pins
InterruptIn Bumper(p8); // External interrupt pin declared as Bumper
Ticker PeriodicInt; // Declare a timer interrupt: PeriodicInt- used for PI
Ticker PeriodicFBInt; //Used for feedback loop


// Declare global variables
int Position;
int sign, t, duty; //int PWMval;
float x=0,x2=0,T,y,ytwo,uProportional,uProportional2;
float uIntegral,uIntegral2,u=0,u2=0, PWMval,PWMval2;
float r,e=0,e2=0, dTime,dTime2, id ,elimit=99,xLimit=0xFFFF;
float xTemp, xTemp2;
float w,w2,Vel,Vel2,N= 16;
float set_point=0,set_point2=0,steerL,steerR;
int Corner_Flag=0;
float L=0.2159; //meters
float rw=0.508; //meters
float theta=0;  // degrees
int dPosition,dPosition2; 
float Pos=0, Pos2=0;
char Direction;


// ********************** Main Thread ***************************************
int main() { // This thread executes first upon reset or power-on.
brake=0;
brake2=0;

PiControllerId = osThreadCreate(osThread(PiControllerThread), NULL);
FeedbackId=osThreadCreate(osThread(FeedbackThread),NULL);

PeriodicInt.attach(&PiControllerISR, 0.001);
PeriodicFBInt.attach(&FeedbackISR,0.05);
//need to do this less frequently than SPI and trggering/echo time.

T=0.001;  // Period in Seconds
y=0;      // Duty Cycle
int16_t id;
id=0x1;

PwmP22.period(T);
PwmP21.period(T);

Bt.printf("Running in Manual Control Mode.\n\r");
Bt.printf("Use W,A,S,D keys to move Forward, Left, Back/Slowdown, and Right\n\r");



SetSPI();

Bt.baud(9600);

id=DE0.write(0x0002); // Specify a read only transactions of 2 words 8402 8002
 Bt.printf("%d\r\n", id); 
    if (id==0x17){ // Checks for good communication
        Bt.printf("Good Communication\n\r");}
    else{Bt.printf("Bad Communication\n\r");}

 
//Loop in main
do {
    
if (Bt.readable()){
    Direction = Bt.getc();
    Bt.putc(Direction);
    
    if(Direction == 'w'){
        set_point+=0.011;
        set_point2+=0.01;
        brake=0;
        brake2=0;
    }
    else if(Direction == 'a'){
        set_point2+=0.01;
        brake=0;
        brake2=0;
        }
        
    else if(Direction == 's'){
        set_point-=0.011;
        set_point2-=0.01;
        brake=0;
        brake2=0;
        }
        
    else if(Direction == 'd'){
        set_point+=0.011;
        brake=0;
        brake2=0;
        }
    else if(Direction =='q'){
        set_point=0;
        set_point2=0;
        brake=1;
        brake2=1;
    }
}
}
while(1);
//end of main loop
}
// ********************end of MAIN!!!!!!! **********************************


// ******** PI Control Tread ****************
void PiControllerThread(void const *argument) {

float uS,uS2,Kp=5, Ki=0.003, uLimit=0.5,i=0.000001,rmax;//0.5
    
    rmax= 0.05;//0.05 0.008
    
    
    
    while (1){
        osSignalWait(SignalPiController, osWaitForever);
        ReadSPIvel();
        led3=!led3; //toggles LED everytime the thread is called
        
        //limit set_points
        if(set_point<=rmax&&set_point>=0){set_point=set_point;}
        else if(set_point<0&&set_point>-rmax){set_point=set_point;}
        else if(set_point>rmax){set_point=rmax;}
        else{set_point=-rmax;}
        
        if(set_point2<=rmax&&set_point2>=0){set_point2=set_point2;} 
        else if(set_point2<0&&set_point2>-rmax){set_point2=set_point2;}
        else if(set_point2>rmax){set_point2=rmax;}
        else{set_point2=-rmax;}

        
        w=Vel;
        w2=-Vel2;
        e=SaturatingSubtract(set_point,w); 
        e2=SaturatingSubtract(set_point2,w2);  
        e=SaturateValue(e,elimit);
        e2=SaturateValue(e2,elimit);
        //e=r-w;
        
        xTemp=SaturatingAddition(x,e);
        xTemp2=SaturatingAddition(x2,e2);
        xTemp=SaturateValue(xTemp, xLimit);
        xTemp2=SaturateValue(xTemp2,xLimit);
        
        //Scaled proportional an  d integral control terms
        uProportional=(Kp*e);
        uProportional2=(Kp*e2);
        uIntegral=(Ki*xTemp);
        uIntegral2=(Ki*xTemp2);
        uS=SaturatingAddition(uProportional,uIntegral);
        uS2=SaturatingAddition(uProportional2,uIntegral2);
        
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
        
        PwmP22.write(y);//left motor
        PwmP21.write(ytwo);//right motor
    }
    //end of while
}// end of PIControlThread

// ******** PiConttroller Timer Interrupt Handler ********
void PiControllerISR(void) {
 // Activate the signal, PiController, with each periodic timer interrupt.
 osSignalSet(PiControllerId,0x1); 
 }


 // *********** IO Reset  *********** //
void IOReset(){
    IoReset=0;
    IoReset=1;
    IoReset=0; 
}
// ********** End IO Reset  ********* //

 
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
 
 
// Saturating Subtract: takes the difference between two inputs 
// and saturates the negative value 
float SaturatingSubtract(float x, float y){
    float z; 
    z =  x - y; // 32-bit overflow detection and saturating arithmetic 
    if((x > 0) && (y < 0) && (z < 0)) z = 0x7FFFFFFF; 
    else if((x < 0) && (y > 0) && (z > 0)) z = 0x80000000; 
    return z;
}
// end of Saturating Subtract


// Saturating addition: takes the difference between two inputs 
// and saturates the negative value 
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
 
 
  // ******** Feedbackthread Timer Interrupt Handler ********
void FeedbackISR(void) {
 // Activate the signal, PiController, with each periodic timer interrupt.
 osSignalSet(FeedbackId,0x1); 
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
          
          
/* if (Obs_Matrix[0]==1||Obs_Matrix[11]==1||Obs_Matrix[1]==1){brake=1,brake2=1;}
            else if (Dist_Matrix[3]>30 || Dist_Matrix[4]>30){
                set_point=set_point2+0.001;
                brake=0;
                brake2=0;
            }
            else {set_point=0.01;set_point2=0.012;Thread::wait(500);}
            //if ((Dist_Matrix[0]<=15)||Corner_Flag){move_to_other_wall();}
    
            */
            
          
    }//end of for get data from sensors
    //Determine_Direction(Obs_Matrix);

        }//end of while
}//end of FeedbackThread
 
 void move_to_other_wall(void){
    // turn in the direction opposite to present wall until at 90 degrees 
    while (theta<90){ //maybe 2*3.14;
        //turn the robot by changing the setpoints
        steerL+=0.000;
        steerR+=0.001;
        //set_point=0.0001;
        //set_point2=0.0001;
        Angle_Update();
        led1=!led1;
        
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
     //identifies if there is an object that is too close (6 cm) 
     unsigned too_close=6; //cm
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