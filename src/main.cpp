#include <Arduino.h>

/////////////////////////////////////////
/* PRE-DEFINED FUNCTIONS AND VARIABLE S*/
#define BAUDRATE  115200 // Hz
#define TSERIAL   100    // ms
#define TSAMPLING 100    // ms: Max 65000*2 ms -> If need longer sampling time, need to config Timer Register at configureRoutineInteruptT()
#define CALIB_POINT_1     0 //g
#define CALIB_POINT_2   100 //g
#define CALIB_POINTS      5
void configureGpio(void);
void configureClk(void);
void configureRoutineInteruptT(void);
void configureAdcLoadcell(void);
void configureHx711(void);
void      measAdc12(void);
uint32_t  measAdcHx711(void);
float getValHx711(uint32_t adcVal, float fSlope, float fOffset);
float getValAdc12(uint16_t adcVal, float fSlope, float fOffset);
void calibLoadcell(void);

volatile uint32_t curCnt = 0;
volatile uint32_t valDLoadcell = 0;
volatile double    valFLoadcell = 0.0f;
volatile double slope = 0, offset = 0;

//////////////////////////////////
/* SETUP HARDWARE CONFIGURATION */
void setup() {
  
  // Init Serial Port with Baudrate
  Serial.begin(BAUDRATE);
  Serial.println(" ");
  Serial.println(">>> Initializing");
  Serial.println(" ");
  delay(1000);

  // Config Function
  configureGpio();
  configureAdcLoadcell();
  configureHx711();
  configureRoutineInteruptT();
  
  // Global interupt Enable + Low Power Mode 0
  _BIS_SR(GIE);
  sleep(1000);

  // Loadcell calibration
  calibLoadcell();

  // Finish Setup Configuration
  Serial.println(" ");
  Serial.println(">>> Start Program"); 
  Serial.println(" ");

}

void configureGpio(void) {
  // Config Output LED at P1.0 Red Led
  P1DIR |= BIT0;    /*Config in/out in Direction bit = 1*/
  P1OUT |= BIT0;    /*Output is High*/

  // Config Output LED at P4.7 Green Led
  P4DIR |= BIT7;    /*Config in/out in Direction bit = 1*/
  P4OUT |= BIT7;    /*Output is High*/

  // Config Input Button at Px.1
  P1DIR &= ~BIT1;   /*Config in/out in Direction bit = 0*/
  P1REN |= BIT1;    /*Enable Register Enable for Pull-Up/Down Register*/
  P1OUT |= BIT1;    /*Config Pull-Up for DI Mode*/
  P1IE  |= BIT1;    /*Config Interupt for P2.1 Button*/
  P1IES |= BIT1;    /*Config Interupt for Falling Edge*/
  P1IFG &= ~BIT1;   /*Clear Interupt Flag at Px.1*/
}

void configureRoutineInteruptT(void) {
  // TA0CTL: Timer-A0 Control Register
  //  TASSEL_2: Source from SMCLK
  //  MC_1:     CountUp to TA0CCR0
  //  TAIE:     Enable Interupt
  TA0CTL |= TASSEL_2 + ID_0 + MC_3 + TAIE;
  TA0CCR0 = (uint16_t) TSAMPLING / 2 * 1000 - 1;
}

void configureAdcLoadcell(void) {
  // Config Pin Analog Input
  P6SEL |= BIT6; // Select periph function on SEL register

  // Config ADC Function
  ADC12CTL0 = ADC12SHT0_2 | ADC12ON;       // Select ADC12 sample and hold time opt = 3 and turn on ADC12
  ADC12CTL1 = ADC12SHP;                    // Select ADC12 sample-and-hold pulse-mode detection.
  ADC12CTL2  |= ADC12RES_2;                // Select ADC12 bit conversion.
  ADC12IE    |= ADC12IE0;                  // Enable interupt on MEM0
  ADC12MCTL0 |= ADC12INCH_6 | ADC12SREF_3; // Select A6 P6.6 and Vref Opt 3 ????????????????????????
}

void configureHx711(void) {
  // P4.1 <-> CLK/OUT
  P4DIR |= BIT1;    /*Config out in Direction bit = 1*/
  P4OUT &= ~BIT1;   /*Output is Low in default*/

  // P4.2 <-> DAT/IN 
  P4DIR &= ~BIT2;   /*Config in Direction bit = 0*/
  P4REN |= BIT2;    /*Enable Register Enable for Pull-Up/Down Register*/
  P4OUT |= BIT2;    /*Config Pull-Up for DI Mode*/
}

void measAdc12(void) { 
  ADC12CTL0 |= ADC12ENC | ADC12SC;  // Enable and start ADC conversion
}

uint32_t measAdcHx711(void) {
  uint8_t  cntAdc = 0;
  uint32_t adcVal = 0;

  // Retrieve 24b DATA from Hx711 
  do {
    // CLK trigger
    P4OUT |= BIT1;
    adcVal = adcVal << 1;
    P4OUT &= ~BIT1;
    if (digitalRead(P4_2)) {
      adcVal++;
    }
    cntAdc++;
  } while (cntAdc < 24);

  P4OUT |= BIT1;
  adcVal = adcVal ^ 0x800000;
  P4OUT &= ~BIT1;

  return adcVal;
}

float getValAdc12(uint16_t adcVal, float fSlope, float fOffset) {
  return (float) adcVal * fSlope + fOffset;
}

float getValHx711(uint32_t adcVal, float fSlope, float fOffset) {
  return (float) adcVal * fSlope + fOffset;
}

void calibLoadcell(void) {
  uint32_t adcVal1[CALIB_POINTS], adcVal2[CALIB_POINTS];
  double   val1 = 0, val2 = 0;

  // Calib at   0g
  Serial.println("  <> Loadcell calib at   0g. Remove Golden Weight from Loadcell.");
  sleep(5000);
  for (int i = CALIB_POINTS-1; i >= 0; i--) {
    
    Serial.print("       (");
    Serial.print(i+1);
    Serial.print(") ");
    Serial.println(valDLoadcell);

    adcVal1[i] = valDLoadcell;

    delay(1000);    
  }

  for (int i = CALIB_POINTS-1; i >= 0; i--) {
    val1 += adcVal1[i];
  }
  val1 = val1 / CALIB_POINTS;

  Serial.println(val1);

  // Calib at 100g
  Serial.println("  <> Loadcell calib at 100g. Put Golden Weight to Loadcell.");
  sleep(5000);
  for (int i = CALIB_POINTS-1; i >= 0; i--) {
    
    Serial.print("       (");
    Serial.print(i+1);
    Serial.print(") ");
    Serial.println(valDLoadcell);

    adcVal2[i] = valDLoadcell;

    delay(1000);
  }

  for (int i = CALIB_POINTS-1; i >= 0; i--) {
    val2 += adcVal2[i];
  }
  val2 = val2 / CALIB_POINTS;

  Serial.println(val2);

  // Calibrating
  slope  = (double) (CALIB_POINT_2 - CALIB_POINT_1) / (val2 - val1);
  offset = (double) (CALIB_POINT_1 - slope * val1);

  Serial.print("  <> Calibration done. Slope = ");
  Serial.print(slope);
  Serial.print(" Offset = ");
  Serial.println(offset);

  delay(1000);
}
////////////////////////////////////
/* MAIN PROGRAM BLOCK - MAIN LOOP */
void loop() {

  /*Sending debug log every seconds*/
  Serial.print(valDLoadcell);
  Serial.print(" --- ");
  Serial.println(valFLoadcell);
  delay(TSERIAL);

}

////////////////////////////
/* INTERUPT PROGRAM BLOCK */
#pragma vector=PORT1_VECTOR
__interrupt void toggleRedLed(void) {
  P1IFG &= ~BIT1;   

  // Toggle off = 1 at P1.0 RED LED
  P1OUT ^= BIT0;
}

#pragma vector=ADC12_VECTOR
__interrupt void adc12bLoadcell(void) {
  ADC12IFG &= ~ADC12IFG0;

  // Get ADC result at Mem0 with 12b-Masked
  valDLoadcell = (uint16_t) ADC12MEM0 & 0x0FFF;
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void timerRoutine(void) {
  TA0CTL &= ~TAIFG;

  // Toggle GREEN LIGHT to indicate operating
  P4OUT ^= BIT7; curCnt++;

  // Main Routine Program
  valDLoadcell = measAdcHx711();
  valFLoadcell = getValHx711(valDLoadcell, slope, offset);
}