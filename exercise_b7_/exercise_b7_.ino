#define readPin A1
#define writePin A0
#define LED_PIN 1
int sampleRate=20000;
int delay_=1000;
int counter;
int zerocounter;
int zero_threshhold=0;
bool interrupt;
bool zero=0;

double frequency;
double period_time;
double deltaT;
int cutoff=50;
float RC;
float alpha;
int filteredVal;
int lastfilteredVal=0;
int minimum=1023;
int maximum=0;
//
//values_[b]=alpha*values[b+1]+(1-alpha)*values[b];


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  analogWriteResolution(10);
  AdcBooster();
}


void loop() {
/*

measure delta T
calibrate filter

*/

//start interrupt
tcConfigure(sampleRate);
tcStartCounter();
delay(5000);
Serial.println("Calibrate deltaT and alpha");
Serial.print("min: ");Serial.println(minimum);
Serial.print("max: ");Serial.println(maximum);
Serial.println("___________");
tcStartCounter();
counter=0;
digitalWrite(LED_PIN, 1);
interrupt=true;
delay(1000);
interrupt=false;
digitalWrite(LED_PIN, 0);
//calculate filter params
deltaT=(1.0)/counter;
RC=1/(2*3.1416*cutoff);
alpha=deltaT/(RC+deltaT);
Serial.print("delta: ");Serial.println(deltaT,10);
Serial.print("alpha: ");Serial.println(alpha,10);

Serial.println("Calibrate min max for zero crossing");
digitalWrite(LED_PIN, 1);
interrupt=true;
delay(1000);
interrupt=false;
digitalWrite(LED_PIN, 0);
zero_threshhold=(maximum-minimum)/2;

Serial.print("min: ");Serial.println(minimum);
Serial.print("max: ");Serial.println(maximum);
Serial.print("zero_threshhold: ");Serial.println(zero_threshhold);
Serial.println(" ");

for(int i=0;i<=100;i++){
counter=0;
interrupt=true;
delay(delay_);
interrupt=false;
if (zerocounter <=(int(sampleRate/50)+100)){
Serial.print("zerocounter: ");Serial.println(zerocounter);}
Serial.print("period_time: ");Serial.println(period_time,6);
Serial.print("frequency: ");Serial.println(frequency,6);
Serial.println(" ");
}

tcDisable();
tcReset();

}


void TC5_Handler (void)
{
if (interrupt==true){
  int sensorVal = analogRead(readPin);
//Serial.println("started"); Serial.println(started);
filteredVal = alpha*float(sensorVal) + (1-alpha)*float(lastfilteredVal);

analogWrite(writePin, filteredVal);
++counter;
if (filteredVal<minimum){minimum=filteredVal;}
else if (filteredVal>maximum){maximum=filteredVal;}
//Serial.print(zero_threshhold !=0);Serial.print(lastfilteredVal<zero_threshhold);Serial.println(filteredVal>zero_threshhold);
//Serial.print(zero_threshhold);Serial.print(" ");Serial.print(lastfilteredVal);Serial.print(" ");Serial.println(filteredVal);


if (zero_threshhold !=0 && lastfilteredVal<zero_threshhold && filteredVal>zero_threshhold){
  if(zero==0){
  zerocounter=counter;
  zero=1;
  }
  else if (zero ==1){
    zerocounter=counter-zerocounter;
    period_time=((zerocounter-1)*deltaT+(deltaT/(filteredVal-lastfilteredVal))*(zero_threshhold-lastfilteredVal));
    frequency=1.0/period_time;
    zero=0;

//    Serial.print("zerocounter: ");Serial.println(zerocounter);
//    Serial.print("filteredVal: ");Serial.println(filteredVal);
//    Serial.print("lastfilteredVal: ");Serial.println(lastfilteredVal);
//    Serial.print("period_time: ");Serial.println(period_time,6);
//    Serial.print("frequency: ");Serial.println(frequency,6);

    }
  }
lastfilteredVal=filteredVal;
}

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
