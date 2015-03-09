// nw2s includes
#include <aJSON.h>
#include <adk.h>
#include <SPI.h>
#include <Wire.h>
#include <IO.h>
using namespace nw2s;

#include <SD.h>

// must uncomment define USING_SERVO_LIB in DueTimer.h
#include <DueTimer.h> 

#define ADC_CH_A 12
#define ADC_CH_A_MSK 0x1000
#define ADC_CH_A_ISR ADC_ISR_EOC12
#define ADC_CH_B 13
#define ADC_CH_B_MSK 0x2000
#define ADC_CH_B_ISR ADC_ISR_EOC13

#define NSAMPLES 20000
#define NDECIMATE 25
#define SAMPERIOD 25 // output to dac every 25us (40KHz)

uint16_t *samplesA;
uint16_t *samplesB;
uint16_t *decimateBuf;

const int clkIn = 46;           // D1 digital clock input
const int recStartIns[2] = {52,53};  // D7,D8
const int playStartIns[2] = {50,51}; // D5,D6
const int writeSDIn = 49; // D4
const int readSDIn = 48; // D3

const int seqOutPin[3] = {37, 35, 33};  // D1, D3, D5 digital outs
const int seqKnobs[3][2] = {{0,1},{2,3},{4,5}}; // A1-A6 control sequences
const int playKnobs[2][2] = {{6,7},{8,9}}; // A7-A10 control sample playback bounds
const int allDOuts[16] = {37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22};
const int allAOuts[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
const int trigTime = 25;  // 25 ms trigger timing

//  interrupt handling of the clock input
volatile int clkState = LOW;

//  controlling the DIO sequencer states
int digState[3] = {LOW, LOW, LOW};        // start with both set low
unsigned long digMilli[3] = {0, 0, 0};  // a place to store millis()
unsigned long deadClockMillis = 0;

// euclidian rhythm settings
int inSteps[3];
int inPulses[3];
int inRotate[3];
unsigned char pulse_tbl[32]  = {1,1,3,1,5,3,7,1,9,5,11,3,13,7,15,1,17,9,19,5,21,11,23,3,25,13,27,7,29,15,31,1};
unsigned char step_tbl[32] = {32,16,32,8,32,16,32,4,32,16,32,8,32,16,32,2,32,16,32,8,32,16,32,4,32,16,32,8,32,16,32,1};

int euArray[3][32];

unsigned long currPulse = 0;
int tmp_stp, tmp_pul, tmp_rot;

int SDEnabled = 1;

void displayBeat(int beat) {
  beat = 15 - beat;
  digitalWrite(45, LOW);
  digitalWrite(41, (1 & beat) ? HIGH : LOW);
  digitalWrite(42, (2 & beat) ? HIGH : LOW);
  digitalWrite(44, (4 & beat) ? HIGH : LOW);
  digitalWrite(43, (8 & beat) ? HIGH : LOW);
  digitalWrite(45, HIGH);
}

void dac_setup () {
  pmc_enable_periph_clk (DACC_INTERFACE_ID);
  DACC->DACC_MR = 
    (1 << 20) | // tag mode on
    (1 << 21); // MAXS mode
  DACC->DACC_IDR = 0xFFFFFFFF; // no interrupts
  DACC->DACC_CHER = DACC_CHER_CH0;   // enable chan0
  DACC->DACC_CHER = DACC_CHER_CH1;    // enable chan1
}

void adc_setup () {
  ADC->ADC_MR &= 0xFFFF00FF; // zero out prescaler
  ADC->ADC_MR |= 0x100; // prescaler = 1
  ADC->ADC_MR |= 0x80; // freerun
  ADC->ADC_IDR = 0xFFFFFFFF;
  ADC->ADC_CHDR = 0xFFFFFFFF;
  NVIC_DisableIRQ(ADC_IRQn);
}
void start_adc_A() {
  ADC->ADC_CHDR = 0xFFFFFFFF;
  ADC->ADC_IDR = 0xFFFFFFFF;
  ADC->ADC_CHER = ADC_CH_A_MSK;
  ADC->ADC_IER = ADC_CH_A_MSK;
  NVIC_EnableIRQ(ADC_IRQn);
}
void start_adc_B() {
  ADC->ADC_CHDR = 0xFFFFFFFF;
  ADC->ADC_IDR = 0xFFFFFFFF;
  ADC->ADC_CHER = ADC_CH_B_MSK;
  ADC->ADC_IER = ADC_CH_B_MSK;
  NVIC_EnableIRQ(ADC_IRQn);
}

volatile int sptr = 0;
volatile int avg = 0;
volatile int subsamples = 0;
volatile int recording = 0;
volatile int playingA = 0;
volatile int playingB = 0;
volatile int startPosA = 0;
volatile int startPosB = 0;
volatile int endPosA = NSAMPLES - 1;
volatile int endPosB = NSAMPLES - 1;

void adc_handle_helper(uint16_t *samples, uint16_t val) {
    avg += val - decimateBuf[sptr]; // boxcar filter decimation
    decimateBuf[sptr] = val;
    if (sptr == (NDECIMATE-1)) {
      samples[subsamples] = avg / NDECIMATE;
      if (subsamples == (NSAMPLES-1)) { // kill recording when we have NSAMPLES
        NVIC_DisableIRQ(ADC_IRQn);
        ADC->ADC_CHDR = 0xFFFFFFFF;
        ADC->ADC_IDR = 0xFFFFFFFF;
        recording = 0;      
      }
      subsamples++;
    }
    sptr = (sptr+1) % NDECIMATE; 
}  

void ADC_Handler (void) {
  if (ADC->ADC_ISR & ADC_CH_A_ISR)
    adc_handle_helper(samplesA, *(ADC->ADC_CDR + ADC_CH_A));
  else if (ADC->ADC_ISR & ADC_CH_B_ISR)
    adc_handle_helper(samplesB, *(ADC->ADC_CDR + ADC_CH_B));
}

volatile int sposA = 0;
void DAC_HandlerA() {
  DACC->DACC_CDR = 0xFFF & samplesA[sposA];
  if (sposA >= endPosA) {
    Timer6.stop();
    playingA = 0;
  } 
  sposA++;
}

volatile int sposB = 0;
void DAC_HandlerB() {
  DACC->DACC_CDR = (1 << 12) | (0xFFF & samplesB[sposB]);
  if (sposB >= endPosB) {
    Timer7.stop();
    playingB = 0;
  } 
  sposB++;
}

void readPlayKnobs() {
  if (!recording) {
    startPosA = (NSAMPLES-1) - round(::analogRead(playKnobs[0][0])/2047.0 * (NSAMPLES-1));
    endPosA = (NSAMPLES-1) - round(::analogRead(playKnobs[0][1])/2047.0 * (NSAMPLES-1));   
    if (endPosA < startPosA) endPosA = startPosA;
    startPosB = (NSAMPLES-1) - round(::analogRead(playKnobs[1][0])/2047.0 * (NSAMPLES-1));
    endPosB = (NSAMPLES-1) - round(::analogRead(playKnobs[1][1])/2047.0 * (NSAMPLES-1)); 
    if (endPosB < startPosB) endPosB = startPosB;
  }
}

void setup() 
{
  samplesA = (uint16_t *) malloc(NSAMPLES * sizeof(uint16_t));
  samplesB = (uint16_t *) malloc(NSAMPLES * sizeof(uint16_t));
  decimateBuf = (uint16_t *) malloc(NDECIMATE * sizeof(uint16_t));
  
  Serial.begin(9600);
  if (!samplesA || !samplesB || !decimateBuf) Serial.println("could not allocate enough mem");
  
  IOUtils::setupPins();
  
  pinMode(clkIn, INPUT);
  pinMode(recStartIns[0], INPUT);
  pinMode(recStartIns[1], INPUT);
  pinMode(playStartIns[0], INPUT);
  pinMode(playStartIns[1], INPUT);
  
  for (int i = 0; i < 16; i++) {
    pinMode(allDOuts[i], OUTPUT);
    digitalWrite(allDOuts[i], LOW);
  }

  // initialize SD card
  if (SDEnabled) {
    pinMode(10, OUTPUT); 
    if (!SD.begin(10)) {
      Serial.println("SD initialization failed!");
      SDEnabled = 0;
    }
  }
  
  readFromSD();

  for (int i=0; i<3; i++) {
    pinMode(seqOutPin[i], OUTPUT);
    digitalWrite(seqOutPin[i], LOW);
  }
  
  
  
  analogReadResolution(12);
  for (int i = 0; i < 3; i++) {
    int a = 31 - round(::analogRead(seqKnobs[i][0])/2047.0 * 31);
    inSteps[i] = step_tbl[a];
    inPulses[i] = pulse_tbl[a];
    inRotate[i] = 31 - round(::analogRead(seqKnobs[i][1])/2047.0 * 31);
    euCalc(i);
  }
  readPlayKnobs();
        
  recording = 1;
  attachInterrupt(clkIn, isr, RISING);
  attachInterrupt(recStartIns[0], startRecA, RISING);
  attachInterrupt(recStartIns[1], startRecB, RISING);
  attachInterrupt(playStartIns[0], startPlayA, RISING);
  attachInterrupt(playStartIns[1], startPlayB, RISING);
  attachInterrupt(readSDIn, readFromSD, RISING);
  attachInterrupt(writeSDIn, writeToSD, RISING);
  dac_setup();
  adc_setup();
  recording = 0;
  
}


void loop() 
{
  
  // check to see if the clock as been set
  if (clkState == HIGH) {
    clkState = LOW;
    if (millis() - deadClockMillis > 500)
      currPulse = 0;
    else
      currPulse++;
    deadClockMillis = millis();

    int outPulse[3] = {0, 0, 0};
    displayBeat(currPulse % 16);
    for (int i=0; i<3; i++) {
      int myPulse = (currPulse - inRotate[i]) % inSteps[i];
      outPulse[i] = euArray[i][myPulse];
    }
    
    for (int i=0; i<3; i++) {
      if (outPulse[i] > 0) {
        digState[i] = HIGH;
        digMilli[i] = millis();
        digitalWrite(seqOutPin[i], HIGH);
      }
    }    
  }
  
  // do we have to turn off any of the digital outputs?
  for (int i=0; i<3; i++) {
    if ((digState[i] == HIGH) && (millis() - digMilli[i] > trigTime)) {
      digState[i] = LOW;
      digitalWrite(seqOutPin[i], LOW);
    }
  }
      
  // reread the inputs in case we need to change
  if (!recording)
  for (int i = 0; i < 3; i++) {
    int a = 31 - round(::analogRead(seqKnobs[i][0])/2047.0 * 31);
    tmp_stp = step_tbl[a];
    tmp_pul = pulse_tbl[a];
    tmp_rot =  31 - round(::analogRead(seqKnobs[i][1])/2047.0 * 31);
    if (tmp_stp != inSteps[i] || tmp_pul != inPulses[i] || tmp_rot != inRotate[i])  {
      inSteps[i] = tmp_stp;
      inPulses[i] = tmp_pul;
      inRotate[i] = tmp_rot;
      euCalc(i);
    }
  }
  
  readPlayKnobs();
  
}

void isr() {
  clkState = HIGH;
}

void startRecA() { 
  if (!recording) { 
    recording = 1;
    subsamples = 0;
    sptr = 0;
    for (int i = 0; i < NDECIMATE; i++) decimateBuf[i] = 0;
    avg = *(ADC->ADC_CDR + ADC_CH_A); // read to clear isr
    avg = 0;
    start_adc_A();    
  } 
}

void startRecB() {
  if (!recording) {
    recording = 1;
    subsamples = 0;
    sptr = 0;
    for (int i = 0; i < NDECIMATE; i++) decimateBuf[i] = 0;
    avg = *(ADC->ADC_CDR + ADC_CH_B); // read to clear isr
    avg = 0;
    start_adc_B();
  } 
}

void startPlayA() {
  if (!playingA) {
    playingA = 1;
    sposA = startPosA;
    Timer6.attachInterrupt(DAC_HandlerA).start(35);
  }  
}

void startPlayB() {
  if (!playingB) {
    playingB = 1;
    sposB = startPosB;
    Timer7.attachInterrupt(DAC_HandlerB).start(35);
  }
}

void writeToSD() {
  if (!SDEnabled) return;
  SD.remove("out.wav");
  File wavOut = SD.open("out.wav", FILE_WRITE);
  if (!wavOut) {
    Serial.println("Error opening out.wav");  
    return;
  }
  wavOut.write((char*) samplesA, NSAMPLES * sizeof(uint16_t));
  wavOut.write((char*) samplesB, NSAMPLES * sizeof(uint16_t));
  wavOut.close();
}

void readFromSD() {
  if (!SDEnabled) return;
  File wavIn = SD.open("out.wav", FILE_READ);
  if (!wavIn) {
    Serial.println("Error opening out.wav");
    return;
  }
  for (int i = 0; i < NSAMPLES; i++)
    samplesA[i] = wavIn.read() | (wavIn.read() << 8);
  for (int i = 0; i < NSAMPLES; i++)
    samplesB[i] = wavIn.read() | (wavIn.read() << 8);
  wavIn.close();  
}


//  euCalc(int) - create a Euclidean Rhythm array.
//
//  NOTE: Thanks to Robin Price for his excellent implementation, and for
//        making the source code available on the Interwebs.
//        For more info, check out: http://crx091081gb.net/
//  ----------------------------------------------------------------------
void euCalc(int ar) {
  int loc = 0;
  
  // clear the array to start
  for (int i=0; i<32; i++) {
    euArray[ar][i] = 0;
  }
  
  if ((inPulses[ar] >= inSteps[ar]) || (inSteps[ar] == 1)) {
        if (inPulses[ar] >= inSteps[ar]) {
            for (int i = 0; i < inSteps[ar]; i++) {
              euArray[ar][loc] = 1;
              loc++;
            }
        }
      } else {
        int offs = inSteps[ar] - inPulses[ar];
        if (offs >= inPulses[ar]) {
            int ppc = offs / inPulses[ar];
            int rmd = offs % inPulses[ar];
            
            for (int i = 0; i < inPulses[ar]; i++) {
              euArray[ar][loc] = 1;
              loc++;
              for (int j = 0; j < ppc; j++) {
                euArray[ar][loc] = 0;
                loc++;
              }
              if (i < rmd) {
                euArray[ar][loc] = 0;
                loc++;
              }
            }
        } else {
          int ppu = (inPulses[ar] - offs) / offs;
          int rmd = (inPulses[ar] - offs) % offs;
            
          for (int i = 0; i < offs; i++) {
            euArray[ar][loc] = 1;
            loc++;
            euArray[ar][loc] = 0;
            loc++;
            for (int j = 0; j < ppu; j++) {
              euArray[ar][loc] = 1;
              loc++;
            }
            if (i < rmd) {
              euArray[ar][loc] = 1;
              loc++;
            }
          }
        }
    }
}


