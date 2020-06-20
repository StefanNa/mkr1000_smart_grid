#define readPin A1
#define writePin A0
#define LED_PIN 1
int sampleRate=20000;
int delay_=1000;
int counter;
int zerocounter;
int zerocounter_;
int zero_threshhold=0;
bool interrupt;
bool zero=0;
int switchcase;

unsigned long start_interrupt;

float frequency;
float period_time;
int factor=10000;
int deltaT;
int cutoff=50;
float RC;
int alpha;
int sensorVal;
int filteredVal=0;
int lastfilteredVal;
int minimum=1023;
int maximum=0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  analogWriteResolution(10);
  AdcBooster();
}


void loop() {
switchcase=0;

//start interrupt
tcConfigure(sampleRate);
delay(2000);
Serial.println("Calibrate deltaT and alpha");
Serial.print("min: ");Serial.println(minimum);
Serial.print("max: ");Serial.println(maximum);
Serial.println("___________");
tcStartCounter();
counter=0;
digitalWrite(LED_PIN, 1);
tcStartCounter();
delay(1000);
tcDisable();
tcReset();
digitalWrite(LED_PIN, 0);
//calculate filter params
deltaT=1000000.0/counter;
RC=1000000.0/(2*3.1416*cutoff);
alpha=deltaT*factor/(RC+deltaT);
Serial.print("rc: ");Serial.println(RC/factor,10);
Serial.print("delta: ");Serial.println(deltaT,10);
Serial.print("alpha: ");Serial.println(alpha/factor,10);Serial.println();

Serial.println("Calibrate min max for zero crossing");
switchcase=1;
digitalWrite(LED_PIN, 1);
tcConfigure(sampleRate);
tcStartCounter();
delay(1000);
tcDisable();
tcReset();
digitalWrite(LED_PIN, 0);
zero_threshhold=(maximum-minimum)/2;

Serial.print("min: ");Serial.println(minimum);
Serial.print("max: ");Serial.println(maximum);
Serial.print("zero_threshhold: ");Serial.println(zero_threshhold);
Serial.println(" ");

Serial.println("Read Frequency for 100 seconds");
switchcase=2;
for(int i=0;i<=10;i++){
zero=0;
tcConfigure(sampleRate);
tcStartCounter();
delay(delay_);
tcDisable();
tcReset();
Serial.print("zerocounter: ");Serial.println(zerocounter_);
Serial.print("period_time: ");Serial.println(period_time,6);
Serial.print("frequency: ");Serial.println(frequency,6);
Serial.println(" ");
}

}


void TC5_Handler (void)
{
  start_interrupt=micros();
  lastfilteredVal=filteredVal;
  sensorVal = analogRead(readPin);
//Serial.println("started"); Serial.println(started);
//filteredVal = alpha*sensorVal + (1-alpha)*lastfilteredVal;
filteredVal = (alpha*sensorVal + (factor-alpha)*lastfilteredVal)/factor;
//Serial.print(sensorVal); Serial.print(" "); Serial.println(filteredVal);
analogWrite(writePin, filteredVal);
++counter;

switch (switchcase) {
    case 1:
      if (filteredVal<minimum){minimum=filteredVal;}
      else if (filteredVal>maximum){maximum=filteredVal;}
      break;
    case 2:
      if (lastfilteredVal<zero_threshhold && filteredVal>=zero_threshhold){
      switch (zero) {
        case 0:
          counter=0;
          zerocounter=counter;
          zero=1;
          break;
        case 1:
          zerocounter_=counter;
          period_time=((counter-1)*deltaT+(deltaT/(filteredVal-lastfilteredVal))*(zero_threshhold-lastfilteredVal));
          frequency=1000000.0/period_time;
//          Serial.println(micros()-start_interrupt);
          if (frequency<48){
          Serial.print(lastfilteredVal);Serial.print(" ");Serial.print(filteredVal);Serial.print(" ");Serial.print(counter);Serial.print(" ");Serial.println(frequency);}
          counter=0;
          break;
    
        }     
      }
      break;   
      }
      


//if (filteredVal<minimum){minimum=filteredVal;}
//else if (filteredVal>maximum){maximum=filteredVal;}
////Serial.print(zero_threshhold !=0);Serial.print(lastfilteredVal<zero_threshhold);Serial.println(filteredVal>zero_threshhold);
////Serial.print(zero_threshhold);Serial.print(" ");Serial.print(lastfilteredVal);Serial.print(" ");Serial.println(filteredVal);
//
//
//if (zero_threshhold !=0 && lastfilteredVal<zero_threshhold && filteredVal>zero_threshhold){
//  if(zero==0){
//  zerocounter=counter;
//  zero=1;
//  }
//  else if (zero ==1){
//    zerocounter=counter-zerocounter-1;
//    period_time=((zerocounter)*deltaT+(deltaT/(filteredVal-lastfilteredVal))*(zero_threshhold-lastfilteredVal));
//    frequency=1.0/period_time;
//    zero=0;
//    Serial.println(micros()-start_interrupt);

//    Serial.print("zerocounter: ");Serial.println(zerocounter);
//    Serial.print("filteredVal: ");Serial.println(filteredVal);
//    Serial.print("lastfilteredVal: ");Serial.println(lastfilteredVal);
//    Serial.print("period_time: ");Serial.println(period_time,6);
//    Serial.print("frequency: ");Serial.println(frequency,6);

//    }
//  }


  TC5->COUNT16.INTFLAG.bit.MC0 = 1;
  
}




void AdcBooster()
{
 ADC->CTRLA.bit.ENABLE = 0;           // Disable ADC
 while( ADC->STATUS.bit.SYNCBUSY == 1 );    // Wait for synchronization
 ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 |  // Divide Clock by 64.
          ADC_CTRLB_RESSEL_10BIT;    // Result on 10 bits
 ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |  // 1 sample
           ADC_AVGCTRL_ADJRES(0x00ul); // Adjusting result by 0
 ADC->SAMPCTRL.reg = 0x00;           // Sampling Time Length = 0
 ADC->CTRLA.bit.ENABLE = 1;           // Enable ADC
 while( ADC->STATUS.bit.SYNCBUSY == 1 );    // Wait for synchronization
} // AdcBooster


void tcConfigure(int sampleRate)
{
 // Enable GCLK for TCC2 and TC5 (timer counter input clock)
 GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
 while (GCLK->STATUS.bit.SYNCBUSY);

 tcReset(); //reset TC5

 // Set Timer counter Mode to 16 bits
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
 // Set TC5 mode as match frequency
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
 //set prescaler and enable TC5
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
 //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
 TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
 while (tcIsSyncing());
 
 // Configure interrupt request
 NVIC_DisableIRQ(TC5_IRQn);
 NVIC_ClearPendingIRQ(TC5_IRQn);
 NVIC_SetPriority(TC5_IRQn, 0);
 NVIC_EnableIRQ(TC5_IRQn);

 // Enable the TC5 interrupt request
 TC5->COUNT16.INTENSET.bit.MC0 = 1;
 while (tcIsSyncing()); //wait until TC5 is done syncing 
} 

bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}
