const int sensorPin = A0;
const int array_size = 1000;
float values[array_size];

void AdcBooster()
{
 ADC->CTRLA.bit.ENABLE = 0;           // Disable ADC
 while( ADC->STATUS.bit.SYNCBUSY == 1 );    // Wait for synchronization
 ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV64 |  // Divide Clock by 64.
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

for (int i = 0; i <= array_size; i++) {
    int sensorVal = analogRead(sensorPin);
    float voltage = (sensorVal / 1024.0) * 3.3;
    values[i]=voltage;
  }
  for (int i = 0; i <= array_size; i++) {
    Serial.println("Viltage: ");
    Serial.println(values[i]);
  }
  delay(3000);

 }
