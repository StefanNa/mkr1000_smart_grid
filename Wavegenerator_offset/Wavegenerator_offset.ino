//Program for generating a 50 Hz sine wave for part A in the DFCR project for Hands-on microcontroller programming
//Program by Stefan Nahstoll & Allan Wandall

volatile int sIndex; //Tracks sinewave points in array
int sampleCount = 100; // Number of samples to read in block
int *wavSamples; //array to store sinewave points
uint32_t sampleRate = 5000; //sample rate of the sine wave
float offsetV = 0.5062; //offset in radians: sin^-1(0.8/(3.3/2)) - offset of 0.8V

//Hz = sampleRate / sampleCount

void setup() {

  analogWriteResolution(10); //set the Arduino DAC for 10 bits of resolution (max)

  /*Allocate the buffer where the samples are stored*/
  wavSamples = (int *) malloc(sampleCount * sizeof(int));
  genSin(sampleCount); //function generates sine wave
  
}

void loop() {
  sIndex = 0;   //Set to zero to start from beginning of waveform
  tcConfigure(sampleRate); //setup the timer counter based off of the user entered sample rate
  //loop until all the sine wave points have been played
  while (sIndex<sampleCount)
  { 
 //start timer, once timer is done interrupt will occur and DAC value will be updated
    tcStartCounter(); 
  }
  //disable and reset timer counter
  tcDisable();
  tcReset();
}

//This function generates a sine wave and stores it in the wavSamples array
//The input argument is the number of points the sine wave is made up of
void genSin(int sCount) {
 const float pi2 = 6.28; //2 x pi
 float in; 
 
 for(int i=0; i<sCount;i++) { //loop to build sine wave based on sample count
  in = pi2*(1/(float)sCount)*(float)i; //calculate value in radians for sin()
  wavSamples[i] = ((int)(sin(in+offsetV)*511.5 + 511.5)); //Calculate sine wave value and offset based on DAC resolution 511.5 = 1023/2
 }
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
  analogWrite(A0, wavSamples[sIndex]);
  sIndex++;
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;
}
