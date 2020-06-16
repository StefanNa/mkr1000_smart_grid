#define readPin A1
#define writePin A0
#define LED_PIN 1
int sampleRate=10;
int delay_=3000;
int counter;
bool interrupt;
bool nofilter;

double deltaT;
int cutoff=50;
float RC;
float alpha;
bool started=0;
float filteredVal;
float lastfilteredVal=0;
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
  // put your main code here, to run repeatedly:
//int sensorVal = analogRead(readPin);
//analogWrite(writePin, sensorVal);
delay(1000);
tcConfigure(sampleRate);
counter=0;
Serial.println("no Filter");
tcStartCounter();
interrupt=true;
nofilter=true;
delay(delay_);
interrupt=false;
nofilter=false;
digitalWrite(LED_PIN, 0);
Serial.println("___________");

deltaT=1.0/counter;
RC=1/(2*3.1416*cutoff);
alpha=deltaT/(RC+deltaT);
Serial.println("Filter 50Hz");
Serial.print("delta: ");Serial.println(deltaT,10);
Serial.print("alpha: ");Serial.println(alpha,10);
interrupt=true;
delay(delay_);
interrupt=false;
digitalWrite(LED_PIN, 0);
Serial.println("___________");

cutoff=10;
deltaT=1.0/counter;
RC=1/(2*3.1416*cutoff);
alpha=deltaT/(RC+deltaT);
Serial.println("Filter 10Hz");
Serial.print("delta: ");Serial.println(deltaT,10);
Serial.print("alpha: ");Serial.println(alpha,10);
started=0;
interrupt=true;
delay(delay_);
interrupt=false;
digitalWrite(LED_PIN, 0);
Serial.println("___________");

cutoff=100;
deltaT=1.0/counter;
RC=1/(2*3.1416*cutoff);
alpha=deltaT/(RC+deltaT);
Serial.println("Filter 100Hz");
Serial.print("delta: ");Serial.println(deltaT,10);
Serial.print("alpha: ");Serial.println(alpha,10);
started=0;
interrupt=true;
delay(delay_);
interrupt=false;
Serial.println("___________");
Serial.println();
tcDisable();
tcReset();

digitalWrite(LED_PIN, 0);
}





void TC5_Handler (void)
{
if (interrupt==true && nofilter==false){
  int sensorVal = analogRead(readPin);
//Serial.println("started"); Serial.println(started);
filteredVal = (!started)*sensorVal + started*alpha*sensorVal + started*(1-alpha)*lastfilteredVal;
lastfilteredVal=filteredVal;
analogWrite(writePin, filteredVal);
digitalWrite(LED_PIN, started);
++counter;
  started=1;
}
else if (nofilter==true){
  int sensorVal = analogRead(readPin);
  analogWrite(writePin, sensorVal);
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
