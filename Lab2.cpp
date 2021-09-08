// C.P. Diduch
// EE4333 Robotics Lab-2, November 29, 2016.
// Template 1 for implementation of a Real Time System
#include "rtos.h"
#include "mbed.h"

// Function prototypes
void WatchdogISR(void const *n);
void WatchdogThread(void const *argument);
void Simple_IO(int led_status);
void Bluetooth_Write();

// Processes and threads
int32_t SignalWatchdog;
osThreadId WatchdogId;
osTimerDef(Wdtimer, WatchdogISR); // Declare a watch dog timer
osThreadDef(WatchdogThread, osPriorityRealtime, DEFAULT_STACK_SIZE); // Declare WatchdogThread as a thread
// osPriorityIdle = -3, ///< priority: idle (lowest)
// osPriorityLow = -2, ///< priority: low
// osPriorityBelowNormal = -1, ///< priority: below normal
// osPriorityNormal = 0, ///< priority: normal (default)
// osPriorityAboveNormal = +1, ///< priority: above normal
// osPriorityHigh = +2, ///< priority: high
// osPriorityRealtime = +3, ///< priority: realtime (highest)

// IO Port Configuration
DigitalOut led1(LED1);
Serial pc(USBTX, USBRX); // Pins (tx, rx) for PC serial channel
// DigitalIn switch_read(p18);
// DigitalOut led_out(p9);
// DigitalOut led_bored(LED2);
// PwmOut PwmP21(p21);
// PwmOut PwmP22(p22);
Serial BluetoothSerial(p28,p27); // Pin 28 Tx, Pain 27 Rx

// ******** Main Thread ********
int main() { // This thread executes first upon reset or power-on.
//char x;
//int led_status=0; led_bored=0;float y, T;
//T=0.001;
//y=T*0.77;
/*
pc.printf("\n\rEnter on-time for followed by enter");
pc.scanf("%f",&y);
pc.printf("\n\n\rPwm period = %f,Pwm on-time = %f", T,
y);
*/
BluetoothSerial.baud(9600);
// Start execution of the thread: WatchdogThread with ID, WatchdogId:
//WatchdogId = osThreadCreate(osThread(WatchdogThread), NULL);
// Start the watch dog timer and enable the watch dog interrupt
//osTimerId OneShot = osTimerCreate(osTimer(Wdtimer), osTimerOnce, (void *)0);
pc.printf("\r\n Hello World - RTOS Template Program");
do {
 //if (pc.readable()){
 //x=pc.getc();
 //pc.putc(x); //Echo keyboard entry
 //}
//Simple_IO(led_status);

//PWM generation

//PwmP21.period(T);
//PwmP22.period(T);
//PwmP21.pulsewidth(y);
//PwmP22.pulsewidth(y);

Bluetooth_Write();
 
 //osTimerStart(OneShot, 2000); // Start or restart the watchdog timer interrupt and set to 2000ms.
 
 //Thread::wait(200); // Go to sleep for 200 ms
}
while(1);
}

//Functions and Threads

//****** Simple IO Function
//void Simple_IO(int led_status){
//   led_status=switch_read;
//   if (led_status){
//   led_out=1;
//   led_bored=!led_bored;
// }
//}

//***Bluetooth write
void Bluetooth_Write(){
 //char x;
 //if (pc.readable()){
 //x=pc.getc();
 pc.putc(pc.getc());
 //BluetoothSerial.putc(pc.getc());
BluetoothSerial.putc(BluetoothSerial.getc());
 //pc.putc(x); //Echo keyboard entry
 //}
 }
 
 


// ******** Watchdog Thread ********
void WatchdogThread(void const *argument) {
while (true) {
 osSignalWait(SignalWatchdog, osWaitForever); // Go to sleep until a signal, SignalWatchdog, is received
 led1 = ~led1;
 }
}
// ******** Watchdog Interrupt Handler ********
void WatchdogISR(void const *n) {
 osSignalSet(WatchdogId,0x1); // Send signal to thread with ID, WatchdogId, i.e., WatchdogThread.
}
