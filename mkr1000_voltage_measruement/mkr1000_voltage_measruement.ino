const int sensorPin = A1;
const int array_size = 3000;
float values[array_size];
//float values_[array_size];
int timer_[array_size];
int time__;
unsigned long startMicros;
unsigned long stopMicros;
double cutoff=50.0/1000000.0;
float RC;
float alpha;

float mean_int(int *x) {
  float n = sizeof(x) / sizeof(x[0]);
  int sum=0;
  for (int i = 0; i < n; i++){
    sum+=x[i];
    return float(sum)/float(n);
    }
  }

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
  // put your setup code here, to run once:
  Serial.begin(9600);
  AdcBooster();
}



void loop() {
  // put your main code here, to run repeatedly:

startMicros=micros();
for (int i = 0; i < array_size; i++) {

    int sensorVal = analogRead(sensorPin);
    float voltage = (sensorVal / 4095.0) * 3.3;
    values[i]=sensorVal;
    stopMicros=micros();
    timer_[i]=stopMicros-startMicros;
    startMicros=stopMicros;
  }
  for (int b = 0; b < array_size-1; b++) {
    RC=1/(2*3.1416*cutoff);
//    Serial.print(cutoff,8);Serial.print("\t");Serial.println(RC,5);
    alpha=timer_[b]/(RC+timer_[b]);
//    alpha=0.01;
//    Serial.print(timer_[b]);Serial.print("\t");Serial.println(alpha,5);
    if (b==0){
//      values_[b]=alpha*values[b+1]+(1-alpha)*values[b];
      }
    else {
//    values_[b]=alpha*values[b+1]+(1-alpha)*values_[b-1];
    }
  }
  for (int a = 0; a <= array_size; a++) {
    
//    Serial.print("Voltage: ");
//    Serial.println(values[a]);
//    Serial.print("Voltage_: ");
    Serial.print(values[a],5);Serial.println(", ");//Serial.println(values_[a]);
//    Serial.print("time: ");
//    Serial.print(sizeof(timer_)/sizeof(timer_[0]));
//    Serial.println(timer_[a]);
    
    
  }
//  Serial.println(mean_int(timer_));
//  Serial.println(timer_[49]);
  delay(1500);
 }
