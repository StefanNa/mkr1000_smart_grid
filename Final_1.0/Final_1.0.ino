#include <LiquidCrystal.h>
#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

#define readPin A1
#define writePin A0
#define LED_PIN 7
#define LED_PIN2 6
#define motorPin A3

const char THING_ID[] = "59279b5b-1d29-4c61-97f2-53033e1f56f4";

const char SSID[]     = "HUAWEI P30";    // Network SSID (name)
const char PASS[]     = "Stefan1995";    // Network password (use for WPA, or use as key for WEP)

//const char SSID[]     = "YI-course";    // Network SSID (name)
//const char PASS[]     = "IloveIoTandArduino12";    // Network password (use for WPA, or use as key for WEP)

float frequencies[100];
int sampleRate=20000;
int delay_=1000; //measuring interval
volatile int counter; // interrupt counter/also for between zero crossings
volatile int zero_counter; //counts zero crossings in timeinterval
volatile int zerocounter_; // used to save the counts at zero crossing to be displayed in void loop()
volatile int zerocounter_max=0;
volatile int zero_threshhold=0; //average value between peaks to detect zero crossings
volatile bool zero=0;
volatile int switchcase;

unsigned long start_interrupt; //time counter for intterrupt
unsigned long operating_time=0; //time counter for intterrupt

volatile float frequency_vol;
float frequency;
volatile float max_freq=0.0;
volatile float period_time;
volatile float subtract=0; //offset at the start of the interrupt to be subtracted
volatile float period_sum; //sum of priod time in interval
volatile int factor=10000; //factor to enable computations with ints instead of floats
volatile int deltaT=50; //time passed per interrupt
int cutoff=50; // low pass wilter cutoff
volatile float RC; // low pass wilter RC
volatile int alpha; // low pass wilter alpha
volatile int sensorVal; //ADC value 0-1023
volatile int filteredVal=0;
volatile int lastfilteredVal;
int minimum=1023; //minimum measured value to be updated as the code runs
int maximum=0; //maximum measured value to be updated as the code runs
float voltageRMS = 0;

LiquidCrystal lcd(1, 0, 5, 4, 3, 2);

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);

void setup() {
  // put your setup code here, to run once:
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("FRQ:");
  lcd.setCursor(0,1);
  lcd.print("RMSV:");
  
  
  Serial.begin(9600);
  delay(500);
  initProperties(); //Initiate the properties for IoT
  ArduinoCloud.begin(ArduinoIoTPreferredConnection); //connect to Arduino IoT cloud
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo(); //Setting up information gain on state of network and errors
  delay(5000);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(motorPin, OUTPUT);
  analogWrite(motorPin, 0);
  
  analogWriteResolution(10);
  AdcBooster(); // speed up adc reading 
}


void loop() {
switchcase=0; // in calibration phase 1 all but measuring the time per interrupt routine is deactivated from the interrupt routine
zero=0;
//start interrupt
tcConfigure(sampleRate); // configure interrupt
delay(2000);
Serial.println("Calibrate deltaT and alpha");
Serial.println("___________");
counter=0; //reset counter
digitalWrite(LED_PIN, HIGH); //turn on LED while interrupt is activated
tcStartCounter(); // start interrupt
delay(1000);
tcDisable(); //disable interrupt
tcReset(); //reset settings
digitalWrite(LED_PIN, LOW);
//calculate filter params
//deltaT=1000000.0/counter; 
RC=1000000.0/(2*3.1416*cutoff);
alpha=deltaT*factor/(RC+deltaT);
Serial.print("rc: ");Serial.println(RC/float(factor),10);
Serial.print("delta: ");Serial.println(deltaT,10);
Serial.print("alpha: ");Serial.println(alpha/float(factor),10);Serial.println();

Serial.println("Calibrate min max for zero crossing");
switchcase=1; //enable first switch case to get min and max values in 
digitalWrite(LED_PIN, HIGH);
tcConfigure(sampleRate);
tcStartCounter();
delay(1000);
tcDisable();
tcReset();
digitalWrite(LED_PIN, LOW);
//calculate threshhold for zero crossing detection
zero_threshhold=(maximum-minimum)/2;

Serial.print("min: ");Serial.println(minimum);
Serial.print("max: ");Serial.println(maximum);
Serial.print("zero_threshhold: ");Serial.println(zero_threshhold);
Serial.println(" ");

Serial.println("Read Frequency for 100 seconds");
//enable interrupt routine to enter frequency measurement
switchcase=2;


for(int i=0;i<100;i++){
period_sum=0;
zero_counter=0;
operating_time=0;
operating_time=0;

zero=0; // parameter to initiate the first measurement 
tcConfigure(sampleRate);
tcStartCounter();
while ( zero_counter<=49){}
//delay(delay_);
tcDisable();
tcReset();
//delay(500);
frequency=zero_counter*1000000/period_sum;
frequencies[i]=frequency;

if (frequency<49.9){
  digitalWrite(LED_PIN2, HIGH);
  analogWrite(motorPin, 0);
}else if(frequency>49.9 && frequency<50.1){
  digitalWrite(LED_PIN2, LOW);
  analogWrite(motorPin, 50*4);
}
else if (frequency>50.1){
  digitalWrite(LED_PIN2, LOW);
  analogWrite(motorPin, 1023);
  }

voltageRMS = (maximum-minimum)*(3.3/1023)*0.3536; //calculate RMS voltage
voltageRMS = 240*0.3536*2; //calculate RMS voltage


lcd.setCursor(6,0);
lcd.print(frequency, DEC);

lcd.setCursor(6,1);
lcd.print(voltageRMS, DEC);

ArduinoCloud.update(); //send updated property information to cloud

Serial.print("zero crossings: ");Serial.println(zero_counter);
Serial.print("period_time 'us': ");Serial.println(period_time,6);
Serial.print("frequency: ");Serial.println(frequency,6);
//Serial.print("zerocounter: ");Serial.println(zerocounter_,6);

//Serial.print("max_operating_time: ");Serial.println(operating_time,6);
//Serial.print("max_frequency: ");Serial.println(max_freq,6);
//Serial.print("zerocounter_max: ");Serial.println(zerocounter_max,6);
max_freq=0;
zerocounter_max=0;
Serial.println(" ");
}

float mean =0;
float var = 0;
for(int i=0;i<100;i++){mean+= frequencies[i];} 
mean=mean/100;
for(int i=0;i<100;i++){var += (frequencies[i] - mean) * (frequencies[i] - mean);}
var /= 100;
float sd = sqrt(var);
Serial.println("");
Serial.println(mean,6);Serial.print(" ");Serial.println(sd,6);
Serial.println("");
while(1){}
}


void TC5_Handler (void)
{
  digitalWrite(LED_PIN, HIGH);
//  start_interrupt=micros();
  lastfilteredVal=filteredVal;
  sensorVal = analogRead(readPin);
filteredVal = (alpha*sensorVal + (factor-alpha)*lastfilteredVal)/factor;
analogWrite(writePin, filteredVal);
++counter;

switch (switchcase) {
    case 1:
      if (filteredVal<minimum){minimum=filteredVal;}
      else if (filteredVal>maximum){maximum=filteredVal;}
      break;
    case 2:
      if (lastfilteredVal<zero_threshhold && filteredVal>zero_threshhold){
      switch (zero) {
        case 0:
          subtract=(deltaT/(filteredVal-lastfilteredVal))*(zero_threshhold-lastfilteredVal);
          counter=0;
          zero=1;
          break;
        case 1:
          zerocounter_=counter;
          period_time=((counter-1)*deltaT+(deltaT/(filteredVal-lastfilteredVal))*(zero_threshhold-lastfilteredVal));//-subtract;
          period_sum +=period_time;
          zero_counter+=1;
//          frequency_vol=1000000.0/period_time;
//          if (frequency_vol>max_freq){max_freq=frequency_vol;zerocounter_max=zerocounter_;}
            
          zero=0;
          break;
    
        }     
      }
      else if (lastfilteredVal<zero_threshhold && filteredVal==zero_threshhold){
      switch (zero) {
        case 0:
          subtract=(deltaT/(filteredVal-lastfilteredVal))*(zero_threshhold-lastfilteredVal);
          counter=0;
          zero=1;
          break;
        case 1:
          zerocounter_=counter;
          period_time=(counter)*deltaT;//-subtract;
//          frequency_vol=1000000.0/period_time;
          period_sum +=period_time;
          zero_counter+=1;
//          if (frequency_vol>max_freq){max_freq=frequency_vol; zerocounter_max=zerocounter_;}
          
//          counter=0;
          zero=0;
          

          break;
    
        }     
      }
      break;   
      }
      

//  if (micros()-start_interrupt>operating_time){operating_time=micros()-start_interrupt;}
  digitalWrite(LED_PIN, LOW);
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

void initProperties(){

  ArduinoCloud.setThingId(THING_ID);
  ArduinoCloud.addProperty(frequency, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(voltageRMS, READ, ON_CHANGE, NULL);

}
