#include <DueTimer.h>
#define Prog_ON
#define Native_ON
void setup() {
  #ifdef Prog_ON
  Serial.begin(115200);
  while (!Serial);
  Serial.println(" Connected ");
  #endif
  #ifdef Native_ON
   SerialUSB.begin(2000000);
    while (!SerialUSB);
  SerialUSB.println(" Native ");
  #endif
  
Timer3.attachInterrupt(Talk).setFrequency(1).start();   //Check to send the buffers if full at a given frequency

}

void loop() {
  
}



void Talk () {
 #ifdef  Prog_ON
  Serial.print(".");
 #endif
 #ifdef Native_ON
  SerialUSB.print("aB");
 #endif
}
