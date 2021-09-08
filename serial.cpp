#include "mbed.h"
 #include "BufferedSerial.h"
 
 BufferedSerial pc(p9, p10);
  BufferedSerial c(USBTX, USBRX);
  
DigitalOut led(LED1);
 int main()
 {
     pc.baud(9600);
     pc.printf("hello\r\n");
     c.printf("hi\r\n");
     while(1)
     {
         led=!led;
         pc.printf("hello\r\n");
         
         
        //pc.putc(pc.getc());
         wait(1);
         /*
         Timer s;
       
         s.start();
         pc.printf("Hello World - buff\n");
         int buffered_time = s.read_us();
         wait(0.1f); // give time for the buffer to empty
       
         s.reset();
         printf("Hello World - poll\n");
         int polled_time = s.read_us();
         s.stop();
         wait(0.1f); // give time for the buffer to empty
       
         pc.printf("printf buffered took %d us\n", buffered_time);
         pc.printf("printf polled took %d us\n", polled_time);
         wait(0.5f);
         */
     }
 }