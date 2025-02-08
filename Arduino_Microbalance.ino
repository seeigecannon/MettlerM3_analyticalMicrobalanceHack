#include <EEPROM.h>
#include <ArduinoSort.h>  // Install "ArduinoSort" library
#define calibrationAddress 0  // Memory address to store the float
#define averageCountAddress 4

volatile uint32_t riseTime = 0;
volatile uint32_t previousRiseTime =0;
volatile uint32_t fallTime = 0;
volatile uint32_t highTime = 0;
volatile uint32_t period = 0;
volatile uint32_t lastCapture = 0;
volatile uint16_t timerOverflowCount = 0;



#define TRIM_PERCENT 20  
#define SAMPLE_SIZE 255 
#define integrationNumber  20

uint32_t dutyCycleSamples[SAMPLE_SIZE];
uint8_t sampleIndex = 0;

uint32_t dutyCycleArray[integrationNumber];
unsigned int arrayPosition = 0;
unsigned long int bufAverageHighTime=0;
unsigned long int bufAveragePeriod=0;
unsigned int bufAverageCount=0;
unsigned int averageCountMax = 2000;
short unsigned int bufArrayPosition=0;

unsigned int timeArrayPosition=0;


//command operations
uint32_t zeroValue=1153537+1160;
bool taring = false;
bool calibrating = false;
unsigned long int commandTimer=0;
int commandArrayPosition=0;
#define zeroSamples 5
#define calibrationSamples 5
uint32_t commandLongAverage[zeroSamples];
uint64_t averageCurrent=0;
uint32_t readingValue=0;
float calibrationValue=35467.3;

String endChar = String(char(0xff))+String(char(0xff))+String(char(0xff));

float preLoadValues[8]={999.9851, 999.9933, 249.9890, 99.9935, 99.990, 199.9893, 99.9928, 249.9919};
float thousands[3]={preLoadValues[0]+preLoadValues[1], preLoadValues[0], 0};
float hundreds[11];


void setup() {
    
    
    calculateOffsets();
    calibrationValue=readFloatFromEEPROM(calibrationAddress);
    EEPROM.get(averageCountAddress, averageCountMax);


    pinMode(8, INPUT);  // ICP1 (Input Capture Pin)
    // Configure Timer1
    TCCR1A = 0;  // Normal mode
    TCCR1B = (1 << ICES1) | (1 << CS10);  // Capture on rising edge, no prescaler (16MHz)
    TIMSK1 = (1 << ICIE1);  // Enable Input Capture Interrupt

    sei();  // Enable global interrupts
    
    
    Serial.begin(9600);  // Start serial communication



}




void loop() {
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();

    if (now - lastUpdate >= 200) {  // Update every 500ms

        averageCurrent = returnSmoothAverage();
        readingValue=calculateMass();
        lastUpdate = now;
    }
  if(Serial.available()){
    char command=Serial.read();
    if(command=='z'){ //z = zero command
      taring=true;
      commandArrayPosition=0;
    }
    if(command=='c'){ //z = zero command
      calibrating=true;
      Serial.println("cal");
      commandArrayPosition=0;
    }
    if(command=='u'){ //z = zero command
      char speed = Serial.read();
      if(speed=='s'){
        averageCountMax=2000;
        EEPROM.put(averageCountAddress, averageCountMax);
      }
      if(speed=='m'){
        averageCountMax=1000;
        EEPROM.put(averageCountAddress, averageCountMax);
      }
      if(speed=='f'){
        averageCountMax=500;
        EEPROM.put(averageCountAddress, averageCountMax);
      }
    }

  }

  if(taring){
    
    if(millis()-commandTimer>averageCountMax){
      commandTimer=millis();
      commandLongAverage[commandArrayPosition]=averageCurrent;
      commandArrayPosition++;
      if(zeroSamples==commandArrayPosition){
        taring=false;
        uint32_t zeroAverage=0;
        for(int i=0; i<zeroSamples; i++){
          zeroAverage+=commandLongAverage[i];
        }
        
        zeroValue=zeroAverage/zeroSamples;

      }
    }

  }
  if(calibrating){
    if(millis()-commandTimer>averageCountMax){
      commandTimer=millis();
      commandLongAverage[commandArrayPosition]=averageCurrent-zeroValue;
      Serial.println("cal");
      commandArrayPosition++;
      if(calibrationSamples==commandArrayPosition){
        calibrating=false;
        float calAverage=0;
        for(int i=0; i<calibrationSamples; i++){
          calAverage+=commandLongAverage[i];
        }
        //Serial.println(calAverage);
        //Serial.println(calibrationSamples);

        calibrationValue=calAverage/(calibrationSamples);
        //Serial.println(calibrationValue);
        calibrationValue=(float)calibrationValue/(float)preLoadValues[4];
        //Serial.println(calibrationValue);
        writeFloatToEEPROM(calibrationAddress, calibrationValue);

      }
    }
  }
}
void writeFloatToEEPROM(int address, float value) {
    EEPROM.put(address, value);
}
float readFloatFromEEPROM(int address) {
    float value;
    EEPROM.get(address, value);
    return value;
}

unsigned long int calculateMass(){
  //Serial.print(averageCurrent);
  //Serial.print(" : ");
  Serial.print("x1.val=");
  Serial.print(averageCurrent+endChar);
  long int zeroCompValue=averageCurrent-zeroValue;
  Serial.print("x0.val=");
  Serial.print(zeroCompValue+endChar);

  float calCompensatedValue=zeroCompValue/calibrationValue;
  Serial.print("x2.val=");
  Serial.print(calCompensatedValue*10000, 0);
  Serial.print(endChar);
  //Serial.println();
}



void calculateOffsets(){
  hundreds[0]=preLoadValues[3]+preLoadValues[4]+preLoadValues[5]+preLoadValues[6]+preLoadValues[8];
}

void insertionSort(uint32_t arr[], int size) {
    for (int i = 1; i < size; i++) {
        uint32_t key = arr[i];
        int j = i - 1;

        // Move elements that are greater than key
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

/*uint32_t returnSmoothAverage(){   //works as of 2-6
  unsigned long int startTime=micros();
  uint32_t copiedArray[integrationNumber];
  for (int i = 0; i < integrationNumber; i++) {
        copiedArray[i] = dutyCycleArray[i];
    }
  insertionSort(copiedArray, integrationNumber);
  unsigned int cutArrayLow = integrationNumber*0.2;
  unsigned int cutArrayHigh = integrationNumber*0.8;
  uint32_t averagedDutyCycle=0;
  for(unsigned int i=cutArrayLow; i<cutArrayHigh; i++){
    averagedDutyCycle+=copiedArray[i];
  }

  averagedDutyCycle = averagedDutyCycle/(cutArrayHigh-cutArrayLow);

  return averagedDutyCycle;

}*/
uint32_t returnSmoothAverage(){   
  //unsigned long int startTime=micros();
  uint32_t copiedArray[integrationNumber];
  for (int i = 0; i < integrationNumber; i++) {
        copiedArray[i] = dutyCycleArray[i];
    }
  insertionSort(copiedArray, integrationNumber);
  unsigned int cutArrayLow = integrationNumber*0.2;
  unsigned int cutArrayHigh = integrationNumber*0.8;
  uint32_t averagedDutyCycle=0;
  for(unsigned int i=cutArrayLow; i<cutArrayHigh; i++){
    averagedDutyCycle+=copiedArray[i];
  }

  averagedDutyCycle = averagedDutyCycle/(cutArrayHigh-cutArrayLow);

  return averagedDutyCycle;

}

//working on 2/6
/*void moveToBuffer(uint32_t highSignal, uint32_t period){  //ISR recording This function takes the highSignal and period and adds them to all of the other highSignals and periods stored. When enough scans have passed, 2000 or so, get the average and store it as a fixed point
  bufAverageHighTime+=highSignal; //add high signal time to buffer
  bufAveragePeriod+=period;   //add period to buffer
  bufAverageCount++;          //advance count
  if(bufAverageCount==averageCountMax){ //generate average if the count gets high enough
    
    bufAverageCount=0;      //reset buffer counter
    uint32_t scale = 100000000;  // Scale factor 
    dutyCycleArray[arrayPosition]=((uint64_t)bufAverageHighTime*scale)/bufAveragePeriod;  //use a fixed point integer to avoid floating point problems

    arrayPosition++;  //select the next index in the storage array
    if(arrayPosition==integrationNumber){   //reset array counter if the array is full
      arrayPosition=0;
    }
    bufAverageHighTime=0;   //reset buffers being added to
    bufAveragePeriod=0;
  }
}*/

void moveToBuffer(uint32_t highSignal, uint32_t period) {
    if (period == 0) return;  // Prevent division by zero

    // Store precomputed duty cycle
    dutyCycleSamples[sampleIndex] = ((uint64_t)highSignal * 100000000ULL) / period;
    sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;

    if (sampleIndex == 0) {  // Process every 100 samples
        sortArray(dutyCycleSamples, SAMPLE_SIZE);

        uint64_t sum = 0;
        uint8_t count = 0;
        
        for (uint8_t i = SAMPLE_SIZE * TRIM_PERCENT / 100; i < SAMPLE_SIZE * (1 - TRIM_PERCENT / 100); i++) {
            sum += dutyCycleSamples[i];
            count++;
        }

        // Store the filtered average duty cycle
        dutyCycleArray[arrayPosition] = sum / count;
        arrayPosition = (arrayPosition + 1) % integrationNumber;
        if(arrayPosition==0){
          sortArray(dutyCycleArray, integrationNumber);
          uint64_t sum = 0;
          uint8_t count = 0;
          for (uint8_t i = integrationNumber * TRIM_PERCENT / 100; i < integrationNumber * (1 - TRIM_PERCENT / 100); i++) {
              sum += dutyCycleSamples[i];
              count++;
          }
          averageCurrent=sum / count;
        }
    }
}

ISR(TIMER1_CAPT_vect) {   //this interrupt fires when the incoming signal is transitioning states
    static bool isRising = true;
    uint32_t currentCapture = ICR1;  // Read the captured time
    uint32_t totalCapture = ((uint32_t)timerOverflowCount << 16) + currentCapture;
    
    if (isRising) {
        previousRiseTime=riseTime;  //save the previous rise time. The current rise time minus the previous rise time equals the period. Note: this needs to be fixed because if the value overflows the previous risetime would be greater than the current risetime
        riseTime=totalCapture;    //get the current time for the interrupt
        TCCR1B &= ~(1 << ICES1);  // Switch to falling edge capture
    } else {
        TCCR1B |= (1 << ICES1);  // Switch back to rising edge
        noInterrupts();
        moveToBuffer(totalCapture-riseTime, riseTime-previousRiseTime);   //save the capture times to the adding buffer when the signal is falling
        interrupts();
    }

    isRising = !isRising;
}

ISR(TIMER1_OVF_vect) {
    timerOverflowCount++;  // Increment on overflow
}
