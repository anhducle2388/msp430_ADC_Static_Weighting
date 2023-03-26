#include <Arduino.h>

/////////////////////////////////////////
/* PRE-DEFINED FUNCTIONS AND VARIABLE S*/
#define BAUDRATE  115200 // Hz
#define TSERIAL   50    // ms
#define TSAMPLING 100    // ms: Max 65000*2 ms -> If need longer sampling time, need to config Timer Register at configureRoutineInteruptT()
#define CALIB_POINT_1        0 //mg
#define CALIB_POINT_2   150000 //mg
#define CALIB_POINTS      5

typedef struct {
  double slope;
  double offset;
  uint32_t valDec;
  double   valDbl;
} calLoadcell;

void configureGpio(void);
void configureClk(void);
void configureRoutineInteruptT(void);
void configureHx711(void);

void      calibLoadcell(calLoadcell *tmpLoadcell);
uint32_t  measAdcHx711_1(calLoadcell *tmpLoadcell);
uint32_t  measAdcHx711_2(calLoadcell *tmpLoadcell);
double    getValHx711(calLoadcell *tmpLoadcell);

volatile uint32_t curCnt = 0;

calLoadcell itemLoadcell_1;
calLoadcell itemLoadcell_2;

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
  configureHx711();
  configureRoutineInteruptT();
  
  // Global interupt Enable + Low Power Mode 0
  _BIS_SR(GIE);
  sleep(1000);

  // Loadcell calibration
  Serial.println(">>> LOADCELL 1");
  Serial.println("");
  calibLoadcell(&itemLoadcell_1);

  Serial.println(">>> LOADCELL 2");
  Serial.println("");
  calibLoadcell(&itemLoadcell_2);

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

void configureHx711(void) {
  // Hx711 - 1st Loadcell
  // P4.1 <-> CLK/OUT
  P4DIR |= BIT1;    /*Config out in Direction bit = 1*/
  P4OUT &= ~BIT1;   /*Output is Low in default*/

  // P4.2 <-> DAT/IN 
  P4DIR &= ~BIT2;   /*Config in Direction bit = 0*/
  P4REN |= BIT2;    /*Enable Register Enable for Pull-Up/Down Register*/
  P4OUT |= BIT2;    /*Config Pull-Up for DI Mode*/

  // Hx711 - 2st Loadcell
  // P6.1 <-> CLK/OUT
  P6DIR |= BIT1;    /*Config out in Direction bit = 1*/
  P6OUT &= ~BIT1;   /*Output is Low in default*/

  // P6.2 <-> DAT/IN 
  P6DIR &= ~BIT2;   /*Config in Direction bit = 0*/
  P6REN |= BIT2;    /*Enable Register Enable for Pull-Up/Down Register*/
  P6OUT |= BIT2;    /*Config Pull-Up for DI Mode*/  
}

uint32_t measAdcHx711_1(calLoadcell * tmpLoadcell) {
  // Hx711 - 1st Loadcell
  uint8_t  cntAdc = 0;
  uint32_t tmpAdcVal = 0;

  do {
    // CLK trigger
    P4OUT |= BIT1;
    tmpAdcVal = tmpAdcVal << 1;
    P4OUT &= ~BIT1;
    if (digitalRead(P4_2)) {
      tmpAdcVal++;
    }
    cntAdc++;
  } while (cntAdc < 24);

  P4OUT |= BIT1;
  tmpAdcVal = tmpAdcVal ^ 0x800000;
  P4OUT &= ~BIT1;
  
  tmpLoadcell->valDec = tmpAdcVal;

  return tmpAdcVal;
}

uint32_t measAdcHx711_2(calLoadcell * tmpLoadcell) {
  // Hx711 - 2nd Loadcell
  uint8_t  cntAdc = 0;
  uint32_t tmpAdcVal = 0;

  do {
    // CLK trigger
    P6OUT |= BIT1;
    tmpAdcVal = tmpAdcVal << 1;
    P6OUT &= ~BIT1;
    if (digitalRead(P6_2)) {
      tmpAdcVal++;
    }
    cntAdc++;
  } while (cntAdc < 24);

  P6OUT |= BIT1;
  tmpAdcVal = tmpAdcVal ^ 0x800000;
  P6OUT &= ~BIT1;

  tmpLoadcell->valDec = tmpAdcVal;
  return tmpAdcVal;
}

double getValHx711(calLoadcell * tmpLoadcell) {
  tmpLoadcell->valDbl = (double) tmpLoadcell->valDec * tmpLoadcell->slope + tmpLoadcell->offset;
  return (double) tmpLoadcell->valDbl;
}

void calibLoadcell(calLoadcell * tmpLoadcell) {
  uint32_t adcVal1[CALIB_POINTS], adcVal2[CALIB_POINTS];
  double   val1 = 0, val2 = 0;

  // Calib at   0g
  Serial.println("  <> Loadcell calib at   0g. Remove Golden Weight within Loadcell in 5s.");
  Serial.print("  <> ");
  for (int i = CALIB_POINTS-1; i >= 0; i--) {
    Serial.print(i+1); Serial.print("  "); sleep(1000);
  };
  Serial.println("");

  for (int i = CALIB_POINTS-1; i >= 0; i--) {
    
    Serial.print("       (");
    Serial.print(i+1);
    Serial.print(") ");
    Serial.println(tmpLoadcell->valDec);

    adcVal1[i] = tmpLoadcell->valDec;

    delay(1000);    
  }

  for (int i = CALIB_POINTS-1; i >= 0; i--) {
    val1 += adcVal1[i];
  }
  val1 = val1 / CALIB_POINTS;

  Serial.println(val1);

  // Calib at 100g
  Serial.println("  <> Loadcell calib at 100g. Put Golden Weight to Loadcell within Loadcell in 5s.");
  Serial.print("  <> ");
  for (int i = CALIB_POINTS-1; i >= 0; i--) {
    Serial.print(i+1); Serial.print("  "); sleep(1000);
  };
  Serial.println("");

  for (int i = CALIB_POINTS-1; i >= 0; i--) {
    
    Serial.print("       (");
    Serial.print(i+1);
    Serial.print(") ");
    Serial.println(tmpLoadcell->valDec);

    adcVal2[i] = tmpLoadcell->valDec;

    delay(1000);
  }

  for (int i = CALIB_POINTS-1; i >= 0; i--) {
    val2 += adcVal2[i];
  }
  val2 = val2 / CALIB_POINTS;

  // Calibrating
  tmpLoadcell->slope  = (double) (CALIB_POINT_2 - CALIB_POINT_1) / (val2 - val1);
  tmpLoadcell->offset = (double) (CALIB_POINT_1 - tmpLoadcell->slope * val1);

  Serial.print("  <> Calibration done. Slope = ");
  Serial.print(tmpLoadcell->slope);
  Serial.print(" Offset = ");
  Serial.println(tmpLoadcell->offset);

  delay(1000);
}
////////////////////////////////////
/* MAIN PROGRAM BLOCK - MAIN LOOP */
void loop() {

  /*Sending debug log every seconds*/
  Serial.print(itemLoadcell_1.valDbl);
  Serial.print(" , ");
  Serial.print(itemLoadcell_2.valDbl);
  Serial.print(" , ");
  Serial.print(itemLoadcell_1.valDbl + itemLoadcell_2.valDbl);
  Serial.println("");
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

#pragma vector=TIMER0_A1_VECTOR
__interrupt void timerRoutine(void) {
  TA0CTL &= ~TAIFG;

  // Toggle GREEN LIGHT to indicate operating
  P4OUT ^= BIT7; curCnt++;

  // Main Routine Program
  measAdcHx711_1(&itemLoadcell_1);
  getValHx711(&itemLoadcell_1);
  measAdcHx711_2(&itemLoadcell_2);
  getValHx711(&itemLoadcell_2);
}