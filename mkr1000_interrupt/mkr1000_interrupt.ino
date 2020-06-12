#include "Timer5.h"

int led = 10;
int hz = 100;
volatile int count=0;

long t = millis();    // timer variable for printout

void setup()
{   
  pinMode(led,OUTPUT);

  // debug output at 115200 baud
  SerialUSB.begin(115200);
  //while (!SerialUSB) ;
    
  SerialUSB.println("starting");

    // define frequency of interrupt
  MyTimer5.begin(hz);  // 200=for toggle every 5msec

    // define the interrupt callback function
    MyTimer5.attachInterrupt(Timer5_IRQ);
  
    // start the timer
    MyTimer5.start();
}

// will be called by the MyTimer5 object
// toggles LED state at pin 10
void Timer5_IRQ(void) {
//    static bool on = false;
    
    count++;  // count number of toggles
    digitalWrite(led, digitalRead(led) ^ 1);
//    if (on == true) {
//      on = false;
//        digitalWrite(led,LOW);
//    } else {
//      on = true;
//        digitalWrite(led,HIGH);
//    }

}

void loop()
{
  // print every 10 secs how often the
    // timer interrupt was called

  while (true) {
    if (millis() - t >= 10000) {
      t = millis();
            SerialUSB.print(millis());
      SerialUSB.print("  count=");
      SerialUSB.println(count);
    }
  }
}
