#include "mbed.h"

DigitalOut led1(LED1);
 
Serial pc(USBTX, USBRX); // tx, rx
Serial BluetoothSerial(p28, p27); // tx, rx

int main(){
char x='a';
BluetoothSerial.baud(9600);

do {
/*
if (pc.readable()) {
x = pc.getc(); // Read keypress from PC
// Read PC keyboard entry and write to BT channel
BluetoothSerial.putc(x); // Send to BT channel
wait(0.04);
pc.putc(x); //Echo keypress back to PC
led=!led;
//wait(1);
}
*/
BluetoothSerial.printf("x");
pc.printf("x");
/*if (BluetoothSerial.readable()) {
x = BluetoothSerial.getc(); // Get char at BT channel
// Read BT device and write PC.
pc.putc(x); // Send to PC
BluetoothSerial.putc(x); //Echo keypress to BT channel
}*/
led1=!led1;
wait_ms(250);
}

while (x != 'q');
}