// C.P. Diduch
// EE4333 Robotics Lab-2, November 29, 2016.
// Template 2 for implementation of a Real Time System

//
#include "rtos.h"
#include "mbed.h"

// Function prototypes
void WatchdogISR(void const *n);
void WatchdogThread(void const *argument);
void ExtInterruptISR(void);
void ExtInterruptThread(void const *argument);
void PeriodicInterruptISR(void);
void PeriodicInterruptThread(void const *argument);

// Processes and threads
int32_t SignalWatchdog, SignalExtInterrupt, SignalPeriodicInterrupt;
osThreadId WatchdogId, ExtInterruptId, PeriodicInterruptId;
osTimerDef(Wdtimer, WatchdogISR); // Declare a watch dog timer
osThreadDef(WatchdogThread, osPriorityRealtime, DEFAULT_STACK_SIZE); // Declare WatchdogThread as a thread/process
osThreadDef(ExtInterruptThread, osPriorityHigh, DEFAULT_STACK_SIZE); // Declare ExtInterruptThread as a thread/process
osThreadDef(PeriodicInterruptThread, osPriorityRealtime, DEFAULT_STACK_SIZE); // Declare PeriodicInterruptThread as thread
// osPriorityIdle = -3, ///< priority: idle (lowest)
// osPriorityLow = -2, ///< priority: low
// osPriorityBelowNormal = -1, ///< priority: below normal
// osPriorityNormal = 0, ///< priority: normal (default)
// osPriorityAboveNormal = +1, ///< priority: above normal
// osPriorityHigh = +2, ///< priority: high
// osPriorityRealtime = +3, ///< priority: realtime (highest)
// IO Port Configuration
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
Serial pc(USBTX, USBRX); // Pins (tx, rx) for PC serial channel
InterruptIn Bumper(p8); // External interrupt pin declared as Bumper
Ticker PeriodicInt; // Declare a timer interrupt: PeriodicInt
// Declare global variables
int Position;

// ******** Main Thread ********
int main() { // This thread executes first upon reset or power-on.
Bumper.rise(&ExtInterruptISR); // Attach the address of the interrupt handler to the rising edge of Bumper
// Start execution of instances of the threads: WatchdogThread with ID, WatchdogId, and thread ExtInterruptThread
// with ID, ExtInterruptId.
WatchdogId = osThreadCreate(osThread(WatchdogThread), NULL);
ExtInterruptId = osThreadCreate(osThread(ExtInterruptThread), NULL);
PeriodicInterruptId = osThreadCreate(osThread(PeriodicInterruptThread), NULL);

// Start the watch dog timer and enable the watch dog interrupt
osTimerId OneShot = osTimerCreate(osTimer(Wdtimer), osTimerOnce, (void *)0);
pc.printf("\r\n Hello World - RTOS Template Program");
PeriodicInt.attach(&PeriodicInterruptISR, .5);

do {
 //if (pc.readable()){
 //x=pc.getc();
 //pc.putc(x); //Echo keyboard entry
 //osTimerStart(OneShot, 2000); // Start or restart the watchdog timer interrupt and set to 2000ms.
 //}
 led4=!led4;
 
Thread:wait(1); // Go to sleep for 500 ms
//pc.printf("\r\n Hello World - RTOS Template Program");
}
while(1);
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
 