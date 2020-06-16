int curVal;
int preVal;

uint32_t sampleRate = 20000; //sample rate of measurement
const int countMax = sampleRate * 2; //determines the amount of samples read before outputting the frequency
unsigned long timer;
const float bitZero = 4095.0/2.0;
unsigned long count;
float zeroTime = 0;
unsigned long zeroCount;
unsigned long periodCount;
float avgTime;


//Function for altering the ADC prescaler and resolution - prescaler: 16; resolution: 12 bit
void AdcBooster()
{
 ADC->CTRLA.bit.ENABLE = 0;           // Disable ADC
 while( ADC->STATUS.bit.SYNCBUSY == 1 );    // Wait for synchronization
 ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 |  // Divide Clock by 64.
          ADC_CTRLB_RESSEL_12BIT;    // Result on 12 bits
 ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |  // 1 sample
           ADC_AVGCTRL_ADJRES(0x00ul); // Adjusting result by 0
 ADC->SAMPCTRL.reg = 0x00;           // Sampling Time Length = 0
 ADC->CTRLA.bit.ENABLE = 1;           // Enable ADC
 while( ADC->STATUS.bit.SYNCBUSY == 1 );    // Wait for synchronization
} // AdcBooster

void setup() {

  //Initialization
  Serial.begin(9600);
  AdcBooster();
  
}

void loop() {
  count = 0;   //Set to zero to start from beginning
  zeroCount = 0;
  tcConfigure(sampleRate); //setup the timer counter based off of the user entered sample rate
  while (zeroTime < (float) countMax) //while loop to start interrupt until the count reaches countMax
  { 
 //start timer, once timer is done interrupt will occur and ADC value will be updated
    tcStartCounter(); 
  }
  //disable and reset timer counter
  tcDisable();
  tcReset();

  //Interpolation at the end
  zeroTime = zeroTime - (curVal - bitZero)/(curVal - preVal);

  //Removing the end-tail of the wave after last rising zero-crossing
  zeroTime = zeroTime - periodCount;

  //The average time between zero crossing is found:
  avgTime = zeroTime/(zeroCount-1);

  //And is printed to serial
  Serial.print("Frequency: ");
  Serial.println(1000000.0/avgTime,4);  

}

// Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
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

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
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

void TC5_Handler (void)
{
  preVal = curVal;
  curVal = analogRead(A1);

  if(periodCount > 30){
    if(preVal < bitZero && curVal > bitZero){
      if(zeroCount == 0){
        zeroTime = 0.0;
        zeroTime = zeroTime - (curVal - bitZero)/(curVal - preVal);
      }
      periodCount = 0;
      zeroCount++;
    }   
  }
  periodCount++;
  zeroTime++;
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;
}
