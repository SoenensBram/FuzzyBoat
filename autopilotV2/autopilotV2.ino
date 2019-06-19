//library loading
#include <Wire.h>
#include <math.h>
#include <Fuzzy.h>
#include <LiquidCrystal_I2C.h>
#include <VarSpeedServo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>

//pin definition
#define RudderPin 11
#define addPin 1
#define subPin 2
#define shiftPin 3
#define entPin 4
#define CSG 7
#define CSM 8
#define encoder A0
#define ledR 5
#define ledG 9
#define ledB 6
#define HIGH 0x1
//#define IRpin 10

// variable definition
float Kset = 0;
float Kact = 0;
int add = 0;
int sub = 0;
int shft = 0;
int ent = 0;
int shftCount = 0;
int lastShft = 0;
int shftOld = 0;
int toggle = 0;
float R = 0; //difference heading and desired
float r = 0; // rate of turn
float delta = 0; //actual rudder angle
float deltac = 0; // desired rudder angle
int encode = 0; //rudder check encoder
int encodeMin = 0;
int encodeMax = 0;
float Magx = 0;
float Magy = 0;

// fuzzy param
const int zero = 0.2;
const int SL = 0.1;
const int SC = 0.4;
const int SM = 1;
const int ML = 0.5;
const int MC = 1;
const int MM = 2;
const int LS = 1.5;
const int LM = MM;
const int LC = 3;

// initiate fuzzy
Fuzzy *fuzzy = new Fuzzy();

//fuzzy memberships
FuzzySet *ZE = new FuzzySet(-zero, 0, 0, zero);  // Instantiating a FuzzySet object
FuzzySet *PS = new FuzzySet(SL, SC, SC, SM);     // Instantiating a FuzzySet object
FuzzySet *PM = new FuzzySet(ML, MC, MC, MM);       // Instantiating a FuzzySet object
FuzzySet *PB = new FuzzySet(LS, LC, 4, 4);       // Instantiating a FuzzySet object
FuzzySet *NS = new FuzzySet(-SM, -SC, -SC, -SL); // Instantiating a FuzzySet object
FuzzySet *NM = new FuzzySet(-MM, -MC, -MC, -ML); // Instantiating a FuzzySet object
FuzzySet *NB = new FuzzySet(-4, -4, -LC, -LS);   // Instantiating a FuzzySet object

FuzzySet *YPS = new FuzzySet(0, 0, 0, 5);     // Instantiating a FuzzySet object
FuzzySet *YPM = new FuzzySet(0, 5, 10, 20);     // Instantiating a FuzzySet object
FuzzySet *YPB = new FuzzySet(15, 30, 40, 40);     // Instantiating a FuzzySet object
FuzzySet *YNS = new FuzzySet(-5, 0, 0, 0); // Instantiating a FuzzySet object
FuzzySet *YNM = new FuzzySet(-20, -10, -5, 0); // Instantiating a FuzzySet object
FuzzySet *YNB = new FuzzySet(-40, -40, -30, -15); // Instantiating a FuzzySet obj
/*
  FuzzySet *YPS = new FuzzySet(sngS, sngS, sngS, sngS);     // Instantiating a FuzzySet object
  FuzzySet *YPM = new FuzzySet(sngM, sngM, sngM, sngM);     // Instantiating a FuzzySet object
  FuzzySet *YPB = new FuzzySet(sngB, sngB, sngB, sngB);     // Instantiating a FuzzySet object
  FuzzySet *YNS = new FuzzySet(-sngS, -sngS, -sngS, -sngS); // Instantiating a FuzzySet object
  FuzzySet *YNM = new FuzzySet(-sngM, -sngM, -sngM, -sngM); // Instantiating a FuzzySet object
  FuzzySet *YNB = new FuzzySet(-sngB, -sngB, -sngB, -sngB); // Instantiating a FuzzySet obj
*/

//Fuzzy array
FuzzySet fuzzyArray0[] = {ZE, PS, PM, PB, NS, NM, NB,} 
FuzzySet fuzzyArray1[] = {YPS, YPM, YPB, YNS, YNM, YNB}

//iterate Fuzzy set instance 12
iterateFuzzySet(arrayToIterate, object12){
  for(int iterate12 = 0; iterate12 < arrayToIterate.size; iterate12++){
    object12->addFuzzySet(arrayToIterate[iterate12]);
  }
}


//lsm9ds1 SPI interface
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(CSG, CSM);
//lcd code
LiquidCrystal_I2C lcd(0x27);
VarSpeedServo Rudder;
void setupSensor() {
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

void setup() {
  Rudder.write(50);
  delay(320);
  encodeMin = analogRead(encoder);
  Rudder.write(130);
  delay(320);
  encodeMax = analogRead(encoder);
  Rudder.write(0);
  map(encode, encodeMin, encodeMax, 50, 130);
  lcd.begin(16, 02); //lcd stup
  Rudder.attach(RudderPin); // servo definition
  Wire.begin(); //SPI start
#ifndef ESP8266
  while (!Serial);
#endif
  Serial.begin(9600);

  if (!lsm.begin()) {
    Serial .println("could not initialise LSM9DS1");
  }
  //course fuzzy memberschip
  FuzzyInput *course = new FuzzyInput(1);          // Instantiating a FuzzyInput object
  iterateFuzzySet(fuzzyArray0, course);
  fuzzy->addFuzzyInput(course);                    // Including the FuzzyInput into Fuzzy

  //ROT fuzzy memberschip
  FuzzyInput *ROT = new FuzzyInput(2);             // Instantiating a FuzzyInput object
  ROT->addFuzzySet(ZE);                            // Including the FuzzySet into FuzzyInput
  ROT->addFuzzySet(PS);                            // Including the FuzzySet into FuzzyInput
  ROT->addFuzzySet(PM);                            // Including the FuzzySet into FuzzyInput
  ROT->addFuzzySet(PB);                            // Including the FuzzySet into FuzzyInput
  ROT->addFuzzySet(NS);                            // Including the FuzzySet into FuzzyInput
  ROT->addFuzzySet(NM);                            // Including the FuzzySet into FuzzyInput
  ROT->addFuzzySet(NB);                            // Including the FuzzySet into FuzzyInput
  fuzzy->addFuzzyInput(course);                    // Including the FuzzyInput into Fuzzy

  //fuzzy output
  FuzzyOutput *y = new FuzzyOutput(1);             // Instantiating a FuzzyOutput objects
  y->addFuzzySet(YPS);                             // Including the FuzzySet into FuzzyOutput
  y->addFuzzySet(YPM);                             // Including the FuzzySet into FuzzyOutput
  y->addFuzzySet(YPB);                             // Including the FuzzySet into FuzzyOutput
  y->addFuzzySet(YNS);                             // Including the FuzzySet into FuzzyOutput
  y->addFuzzySet(YNM);                             // Including the FuzzySet into FuzzyOutput
  y->addFuzzySet(YNB);                             // Including the FuzzySet into FuzzyOutput
  fuzzy->addFuzzyOutput(y);                        // Including the FuzzyOutput into Fuzzy


  // Building FuzzyRule "IF course and ROT THEN y"
  //ROT=NB
  FuzzyRuleAntecedent *ifCourseNBAndROTNB = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NB, NB);
  FuzzyRuleAntecedent *ifCourseNMAndROTNB = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NM, NB);
  FuzzyRuleAntecedent *ifCourseNSAndROTNB = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NS, NB);
  FuzzyRuleAntecedent *ifCourseZEAndROTNB = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(ZE, NB);
  FuzzyRuleAntecedent *ifCoursePSAndROTNB = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PS, NB);
  FuzzyRuleAntecedent *ifCoursePMAndROTNB = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PM, NB);
  FuzzyRuleAntecedent *ifCoursePBAndROTNB = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PB, NB);
  //ROT=NM
  FuzzyRuleAntecedent * ifCourseNBAndROTNM = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NB, NM);
  FuzzyRuleAntecedent * ifCourseNMAndROTNM = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NM, NM);
  FuzzyRuleAntecedent *ifCourseNSAndROTNM = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NS, NM);
  FuzzyRuleAntecedent *ifCourseZEAndROTNM = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(ZE, NM);
  FuzzyRuleAntecedent *ifCoursePSAndROTNM = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PS, NM);
  FuzzyRuleAntecedent *ifCoursePMAndROTNM = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PM, NM);
  FuzzyRuleAntecedent *ifCoursePBAndROTNM = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PB, NM);
  //ROT=NS
  FuzzyRuleAntecedent *ifCourseNBAndROTNS = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NB, NS);
  FuzzyRuleAntecedent *ifCourseNMAndROTNS = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NM, NS);
  FuzzyRuleAntecedent *ifCourseNSAndROTNS = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NS, NS);
  FuzzyRuleAntecedent *ifCourseZEAndROTNS = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(ZE, NS);
  FuzzyRuleAntecedent *ifCoursePSAndROTNS = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PS, NS);
  FuzzyRuleAntecedent *ifCoursePMAndROTNS = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PM, NS);
  FuzzyRuleAntecedent *ifCoursePBAndROTNS = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PB, NS);
  //ROT=ZE
  FuzzyRuleAntecedent *ifCourseNBAndROTZE = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NB, ZE);
  FuzzyRuleAntecedent *ifCourseNMAndROTZE = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NM, ZE);
  FuzzyRuleAntecedent *ifCourseNSAndROTZE = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NS, ZE);
  FuzzyRuleAntecedent *ifCourseZEAndROTZE = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(ZE, ZE);
  FuzzyRuleAntecedent *ifCoursePSAndROTZE = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PS, ZE);
  FuzzyRuleAntecedent *ifCoursePMAndROTZE = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PM, ZE);
  FuzzyRuleAntecedent *ifCoursePBAndROTZE = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PB, ZE);
  //rOT=PS
  FuzzyRuleAntecedent *ifCourseNBAndROTPS = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NB, PS);
  FuzzyRuleAntecedent *ifCourseNMAndROTPS = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NM, PS);
  FuzzyRuleAntecedent *ifCourseNSAndROTPS = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NS, PS);
  FuzzyRuleAntecedent *ifCourseZEAndROTPS = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(ZE, PS);
  FuzzyRuleAntecedent *ifCoursePSAndROTPS = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PS, PS);
  FuzzyRuleAntecedent *ifCoursePMAndROTPS = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PM, PS);
  FuzzyRuleAntecedent *ifCoursePBAndROTPS = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PB, PS);
  //ROT=PM
  FuzzyRuleAntecedent *ifCourseNBAndROTPM = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NB, PM);
  FuzzyRuleAntecedent *ifCourseNMAndROTPM = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NM, PM);
  FuzzyRuleAntecedent *ifCourseNSAndROTPM = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NS, PM);
  FuzzyRuleAntecedent *ifCourseZEAndROTPM = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(ZE, PM);
  FuzzyRuleAntecedent *ifCoursePSAndROTPM = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PS, PM);
  FuzzyRuleAntecedent *ifCoursePMAndROTPM = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PM, PM);
  FuzzyRuleAntecedent *ifCoursePBAndROTPM = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PB, PM);
  //ROT=PB
  FuzzyRuleAntecedent *ifCourseNBAndROTPB = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NB, PB);
  FuzzyRuleAntecedent *ifCourseNMAndROTPB = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NM, PB);
  FuzzyRuleAntecedent *ifCourseNSAndROTPB = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(NS, PB);
  FuzzyRuleAntecedent *ifCourseZEAndROTPB = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(ZE, PB);
  FuzzyRuleAntecedent *ifCoursePSAndROTPB = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PS, PB);
  FuzzyRuleAntecedent *ifCoursePMAndROTPB = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PM, PB);
  FuzzyRuleAntecedent *ifCoursePBAndROTPB = new FuzzyRuleAntecedent();
  ifCourseNBAndROTNB->joinWithAND(PB, PB);


  /*output consequent



  */
  FuzzyRuleConsequent *thenCourseNBAndROTNB = new FuzzyRuleConsequent();
  thenCourseNBAndROTNB->addOutput(NB);
  FuzzyRuleConsequent *thenCourseNMAndROTNB = new FuzzyRuleConsequent();
  thenCourseNMAndROTNB->addOutput(NB);
  FuzzyRuleConsequent *thenCourseNSAndROTNB = new FuzzyRuleConsequent();
  thenCourseNSAndROTNB->addOutput(NB);
  FuzzyRuleConsequent *thenCourseZEAndROTNB = new FuzzyRuleConsequent();
  thenCourseZEAndROTNB->addOutput(NB);
  FuzzyRuleConsequent *thenCoursePSAndROTNB = new FuzzyRuleConsequent();
  thenCoursePSAndROTNB->addOutput(NM);
  FuzzyRuleConsequent *thenCoursePMAndROTNB = new FuzzyRuleConsequent();
  thenCoursePMAndROTNB->addOutput(NS);
  FuzzyRuleConsequent *thenCoursePBAndROTNB = new FuzzyRuleConsequent();
  thenCoursePBAndROTNB->addOutput(ZE);

  FuzzyRuleConsequent *thenCourseNBAndROTNM = new FuzzyRuleConsequent();
  thenCourseNBAndROTNM->addOutput(NB);
  FuzzyRuleConsequent *thenCourseNMAndROTNM = new FuzzyRuleConsequent();
  thenCourseNBAndROTNM->addOutput(NB);
  FuzzyRuleConsequent *thenCourseNSAndROTNM = new FuzzyRuleConsequent();
  thenCourseNBAndROTNM->addOutput(NB);
  FuzzyRuleConsequent *thenCourseZEAndROTNM = new FuzzyRuleConsequent();
  thenCourseNBAndROTNM->addOutput(NM);
  FuzzyRuleConsequent *thenCoursePSAndROTNM = new FuzzyRuleConsequent();
  thenCourseNBAndROTNM->addOutput(NS);
  FuzzyRuleConsequent *thenCoursePMAndROTNM = new FuzzyRuleConsequent();
  thenCourseNBAndROTNM->addOutput(ZE);
  FuzzyRuleConsequent *thenCoursePBAndROTNM = new FuzzyRuleConsequent();
  thenCourseNBAndROTNM->addOutput(PS);

  FuzzyRuleConsequent *thenCourseNBAndROTNS = new FuzzyRuleConsequent();
  thenCourseNBAndROTNS->addOutput(NB);
  FuzzyRuleConsequent *thenCourseNMAndROTNS = new FuzzyRuleConsequent();
  thenCourseNBAndROTNS->addOutput(NB);
  FuzzyRuleConsequent *thenCourseNSAndROTNS = new FuzzyRuleConsequent();
  thenCourseNBAndROTNS->addOutput(NM);
  FuzzyRuleConsequent *thenCourseZEAndROTNS = new FuzzyRuleConsequent();
  thenCourseNBAndROTNS->addOutput(NS);
  FuzzyRuleConsequent *thenCoursePSAndROTNS = new FuzzyRuleConsequent();
  thenCourseNBAndROTNS->addOutput(ZE);
  FuzzyRuleConsequent *thenCoursePMAndROTNS = new FuzzyRuleConsequent();
  thenCourseNBAndROTNS->addOutput(PS);
  FuzzyRuleConsequent *thenCoursePBAndROTNS = new FuzzyRuleConsequent();
  thenCourseNBAndROTNS->addOutput(PM);

  FuzzyRuleConsequent *thenCourseNBAndROTZE = new FuzzyRuleConsequent();
  thenCourseNBAndROTZE->addOutput(NB);
  FuzzyRuleConsequent *thenCourseNMAndROTZE = new FuzzyRuleConsequent();
  thenCourseNBAndROTZE->addOutput(NM);
  FuzzyRuleConsequent *thenCourseNSAndROTZE = new FuzzyRuleConsequent();
  thenCourseNBAndROTZE->addOutput(NS);
  FuzzyRuleConsequent *thenCourseZEAndROTZE = new FuzzyRuleConsequent();
  thenCourseNBAndROTZE->addOutput(ZE);
  FuzzyRuleConsequent *thenCoursePSAndROTZE = new FuzzyRuleConsequent();
  thenCourseNBAndROTZE->addOutput(PS);
  FuzzyRuleConsequent *thenCoursePMAndROTZE = new FuzzyRuleConsequent();
  thenCourseNBAndROTZE->addOutput(PM);
  FuzzyRuleConsequent *thenCoursePBAndROTZE = new FuzzyRuleConsequent();
  thenCourseNBAndROTZE->addOutput(PB);

  FuzzyRuleConsequent *thenCourseNBAndROTPS = new FuzzyRuleConsequent();
  thenCourseNBAndROTPS->addOutput(NM);
  FuzzyRuleConsequent *thenCourseNMAndROTPS = new FuzzyRuleConsequent();
  thenCourseNBAndROTPS->addOutput(NS);
  FuzzyRuleConsequent *thenCourseNSAndROTPS = new FuzzyRuleConsequent();
  thenCourseNBAndROTPS->addOutput(ZE);
  FuzzyRuleConsequent *thenCourseZEAndROTPS = new FuzzyRuleConsequent();
  thenCourseNBAndROTPS->addOutput(PS);
  FuzzyRuleConsequent *thenCoursePSAndROTPS = new FuzzyRuleConsequent();
  thenCourseNBAndROTPS->addOutput(PM);
  FuzzyRuleConsequent *thenCoursePMAndROTPS = new FuzzyRuleConsequent();
  thenCourseNBAndROTPS->addOutput(PB);
  FuzzyRuleConsequent *thenCoursePBAndROTPS = new FuzzyRuleConsequent();
  thenCourseNBAndROTPS->addOutput(PB);

  FuzzyRuleConsequent *thenCourseNBAndROTPM = new FuzzyRuleConsequent();
  thenCourseNBAndROTPM->addOutput(NS);
  FuzzyRuleConsequent *thenCourseNMAndROTPM = new FuzzyRuleConsequent();
  thenCourseNBAndROTPM->addOutput(ZE);
  FuzzyRuleConsequent *thenCourseNSAndROTPM = new FuzzyRuleConsequent();
  thenCourseNBAndROTPM->addOutput(PS);
  FuzzyRuleConsequent *thenCourseZEAndROTPM = new FuzzyRuleConsequent();
  thenCourseNBAndROTPM->addOutput(PM);
  FuzzyRuleConsequent *thenCoursePSAndROTPM = new FuzzyRuleConsequent();
  thenCourseNBAndROTPM->addOutput(PB);
  FuzzyRuleConsequent *thenCoursePMAndROTPM = new FuzzyRuleConsequent();
  thenCourseNBAndROTPM->addOutput(PB);
  FuzzyRuleConsequent *thenCoursePBAndROTPM = new FuzzyRuleConsequent();
  thenCourseNBAndROTPM->addOutput(PB);

  FuzzyRuleConsequent *thenCourseNBAndROTPB = new FuzzyRuleConsequent();
  thenCourseNBAndROTPB->addOutput(ZE);
  FuzzyRuleConsequent *thenCourseNMAndROTPB = new FuzzyRuleConsequent();
  thenCourseNBAndROTPB->addOutput(PS);
  FuzzyRuleConsequent *thenCourseNSAndROTPB = new FuzzyRuleConsequent();
  thenCourseNBAndROTPB->addOutput(PM);
  FuzzyRuleConsequent *thenCourseZEAndROTPB = new FuzzyRuleConsequent();
  thenCourseNBAndROTPB->addOutput(PB);
  FuzzyRuleConsequent *thenCoursePSAndROTPB = new FuzzyRuleConsequent();
  thenCourseNBAndROTPB->addOutput(PB);
  FuzzyRuleConsequent *thenCoursePMAndROTPB = new FuzzyRuleConsequent();
  thenCourseNBAndROTPB->addOutput(PB);
  FuzzyRuleConsequent *thenCoursePBAndROTPB = new FuzzyRuleConsequent();
  thenCourseNBAndROTPB->addOutput(PB);

  //fuzzy rule assembly
  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, ifCourseNBAndROTNB , thenCourseNBAndROTNB);
  fuzzy->addFuzzyRule(fuzzyRule1);
  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, ifCourseNBAndROTNM , thenCourseNBAndROTNM);
  fuzzy->addFuzzyRule(fuzzyRule2);
  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, ifCourseNBAndROTNS , thenCourseNBAndROTNS);
  fuzzy->addFuzzyRule(fuzzyRule3);
  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, ifCourseNBAndROTZE , thenCourseNBAndROTZE);
  fuzzy->addFuzzyRule(fuzzyRule4);
  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, ifCourseNBAndROTPS , thenCourseNBAndROTPS);
  fuzzy->addFuzzyRule(fuzzyRule5);
  FuzzyRule *fuzzyRule6 = new FuzzyRule(6, ifCourseNBAndROTPM , thenCourseNBAndROTPM);
  fuzzy->addFuzzyRule(fuzzyRule6);
  FuzzyRule *fuzzyRule7 = new FuzzyRule(7, ifCourseNBAndROTPB , thenCourseNBAndROTPB);
  fuzzy->addFuzzyRule(fuzzyRule7);

  FuzzyRule *fuzzyRule8 = new FuzzyRule(8, ifCourseNMAndROTNB , thenCourseNMAndROTNB);
  fuzzy->addFuzzyRule(fuzzyRule8);
  FuzzyRule *fuzzyRule9 = new FuzzyRule(9, ifCourseNMAndROTNM , thenCourseNMAndROTNM);
  fuzzy->addFuzzyRule(fuzzyRule9);
  FuzzyRule *fuzzyRule10 = new FuzzyRule(10, ifCourseNMAndROTNS , thenCourseNMAndROTNS);
  fuzzy->addFuzzyRule(fuzzyRule10);
  FuzzyRule *fuzzyRule11 = new FuzzyRule(11, ifCourseNMAndROTZE , thenCourseNMAndROTZE);
  fuzzy->addFuzzyRule(fuzzyRule11);
  FuzzyRule *fuzzyRule12 = new FuzzyRule(12, ifCourseNMAndROTPS , thenCourseNMAndROTPS);
  fuzzy->addFuzzyRule(fuzzyRule12);
  FuzzyRule *fuzzyRule13 = new FuzzyRule(13, ifCourseNMAndROTPM , thenCourseNMAndROTPM);
  fuzzy->addFuzzyRule(fuzzyRule13);
  FuzzyRule *fuzzyRule14 = new FuzzyRule(14, ifCourseNMAndROTPB , thenCourseNMAndROTPB);
  fuzzy->addFuzzyRule(fuzzyRule14);

  FuzzyRule *fuzzyRule15 = new FuzzyRule(15, ifCourseNSAndROTNB , thenCourseNSAndROTNB);
  fuzzy->addFuzzyRule(fuzzyRule15);
  FuzzyRule *fuzzyRule16 = new FuzzyRule(16, ifCourseNSAndROTNM , thenCourseNSAndROTNM);
  fuzzy->addFuzzyRule(fuzzyRule16);
  FuzzyRule *fuzzyRule17 = new FuzzyRule(17, ifCourseNSAndROTNS , thenCourseNSAndROTNS);
  fuzzy->addFuzzyRule(fuzzyRule17);
  FuzzyRule *fuzzyRule18 = new FuzzyRule(18, ifCourseNSAndROTZE , thenCourseNSAndROTZE);
  fuzzy->addFuzzyRule(fuzzyRule18);
  FuzzyRule *fuzzyRule19 = new FuzzyRule(19, ifCourseNSAndROTPS , thenCourseNSAndROTPS);
  fuzzy->addFuzzyRule(fuzzyRule19);
  FuzzyRule *fuzzyRule20 = new FuzzyRule(20, ifCourseNSAndROTPM , thenCourseNSAndROTPM);
  fuzzy->addFuzzyRule(fuzzyRule20);
  FuzzyRule *fuzzyRule21 = new FuzzyRule(21, ifCourseNSAndROTPB , thenCourseNSAndROTPB);
  fuzzy->addFuzzyRule(fuzzyRule21);

  FuzzyRule *fuzzyRule22 = new FuzzyRule(22, ifCourseZEAndROTNB , thenCourseZEAndROTNB);
  fuzzy->addFuzzyRule(fuzzyRule22);
  FuzzyRule *fuzzyRule23 = new FuzzyRule(23, ifCourseZEAndROTNM , thenCourseZEAndROTNM);
  fuzzy->addFuzzyRule(fuzzyRule23);
  FuzzyRule *fuzzyRule24 = new FuzzyRule(24, ifCourseZEAndROTNS , thenCourseZEAndROTNS);
  fuzzy->addFuzzyRule(fuzzyRule24);
  FuzzyRule *fuzzyRule25 = new FuzzyRule(25, ifCourseZEAndROTZE , thenCourseZEAndROTZE);
  fuzzy->addFuzzyRule(fuzzyRule25);
  FuzzyRule *fuzzyRule26 = new FuzzyRule(26, ifCourseZEAndROTPS , thenCourseZEAndROTPS);
  fuzzy->addFuzzyRule(fuzzyRule26);
  FuzzyRule *fuzzyRule27 = new FuzzyRule(27, ifCourseZEAndROTPM , thenCourseZEAndROTPM);
  fuzzy->addFuzzyRule(fuzzyRule27);
  FuzzyRule *fuzzyRule28 = new FuzzyRule(28, ifCourseZEAndROTPB , thenCourseZEAndROTPB);
  fuzzy->addFuzzyRule(fuzzyRule28);

  FuzzyRule *fuzzyRule29 = new FuzzyRule(29, ifCoursePSAndROTNB , thenCoursePSAndROTNB);
  fuzzy->addFuzzyRule(fuzzyRule29);
  FuzzyRule *fuzzyRule30 = new FuzzyRule(30, ifCoursePSAndROTNM , thenCoursePSAndROTNM);
  fuzzy->addFuzzyRule(fuzzyRule30);
  FuzzyRule *fuzzyRule31 = new FuzzyRule(31, ifCoursePSAndROTNS , thenCoursePSAndROTNS);
  fuzzy->addFuzzyRule(fuzzyRule31);
  FuzzyRule *fuzzyRule32 = new FuzzyRule(32, ifCoursePSAndROTZE , thenCoursePSAndROTZE);
  fuzzy->addFuzzyRule(fuzzyRule32);
  FuzzyRule *fuzzyRule33 = new FuzzyRule(33, ifCoursePSAndROTPS , thenCoursePSAndROTPS);
  fuzzy->addFuzzyRule(fuzzyRule33);
  FuzzyRule *fuzzyRule34 = new FuzzyRule(34, ifCoursePSAndROTPM , thenCoursePSAndROTPM);
  fuzzy->addFuzzyRule(fuzzyRule34);
  FuzzyRule *fuzzyRule35 = new FuzzyRule(35, ifCoursePSAndROTPB , thenCoursePSAndROTPB);
  fuzzy->addFuzzyRule(fuzzyRule35);

  FuzzyRule *fuzzyRule36 = new FuzzyRule(36, ifCoursePMAndROTNB , thenCoursePMAndROTNB);
  fuzzy->addFuzzyRule(fuzzyRule36);
  FuzzyRule *fuzzyRule37 = new FuzzyRule(37, ifCoursePMAndROTNM , thenCoursePMAndROTNM);
  fuzzy->addFuzzyRule(fuzzyRule37);
  FuzzyRule *fuzzyRule38 = new FuzzyRule(38, ifCoursePMAndROTNS , thenCoursePMAndROTNS);
  fuzzy->addFuzzyRule(fuzzyRule38);
  FuzzyRule *fuzzyRule39 = new FuzzyRule(39, ifCoursePMAndROTZE , thenCoursePMAndROTZE);
  fuzzy->addFuzzyRule(fuzzyRule39);
  FuzzyRule *fuzzyRule40 = new FuzzyRule(40, ifCoursePMAndROTPS , thenCoursePMAndROTPS);
  fuzzy->addFuzzyRule(fuzzyRule40);
  FuzzyRule *fuzzyRule41 = new FuzzyRule(41, ifCoursePMAndROTPM , thenCoursePMAndROTPM);
  fuzzy->addFuzzyRule(fuzzyRule41);
  FuzzyRule *fuzzyRule42 = new FuzzyRule(42, ifCoursePMAndROTPB , thenCoursePMAndROTPB);
  fuzzy->addFuzzyRule(fuzzyRule42);

  FuzzyRule *fuzzyRule43 = new FuzzyRule(43, ifCoursePBAndROTNB , thenCoursePBAndROTNB);
  fuzzy->addFuzzyRule(fuzzyRule43);
  FuzzyRule *fuzzyRule44 = new FuzzyRule(44, ifCoursePBAndROTNM , thenCoursePBAndROTNM);
  fuzzy->addFuzzyRule(fuzzyRule44);
  FuzzyRule *fuzzyRule45 = new FuzzyRule(45, ifCoursePBAndROTNS , thenCoursePBAndROTNS);
  fuzzy->addFuzzyRule(fuzzyRule45);
  FuzzyRule *fuzzyRule46 = new FuzzyRule(46, ifCoursePBAndROTZE , thenCoursePBAndROTZE);
  fuzzy->addFuzzyRule(fuzzyRule46);
  FuzzyRule *fuzzyRule47 = new FuzzyRule(47, ifCoursePBAndROTPS , thenCoursePBAndROTPS);
  fuzzy->addFuzzyRule(fuzzyRule47);
  FuzzyRule *fuzzyRule48 = new FuzzyRule(48, ifCoursePBAndROTPM , thenCoursePBAndROTPM);
  fuzzy->addFuzzyRule(fuzzyRule48);
  FuzzyRule *fuzzyRule49 = new FuzzyRule(49, ifCoursePBAndROTPB , thenCoursePBAndROTPB);
  fuzzy->addFuzzyRule(fuzzyRule49);
}

void loop() {
  //read values
  lsm.read();
  sensors_event_t a, m, g, temp; //get event acceleration, magnetic and gyro
  lsm.getEvent(&a, &m, &g, &temp);
  add = digitalRead(addPin);
  sub = digitalRead(subPin);
  shft = digitalRead(shiftPin);
  ent = digitalRead(entPin);
  encode = analogRead(encoder);

  Magx = m.magnetic.x;
  Magy = -m.magnetic.y;
  Kact = atan2(Magy, Magx);
  Kact = Kact -= 0.017; // correctie declinatie
  Kact = Kact * 180 / PI;
  if (Kact < 0) {
    Kact += 360;
  }
  Kact = 360 - Kact;
  r =  g.gyro.x;

  if (shft == 0 && lastShft == 1) //start instellen
  { digitalWrite(ledR, 255);
    digitalWrite(ledG, 255);
    do {

      if (shft != shftOld) {
        if (shft == HIGH) {
          shftCount++;
        }
        switch (shftCount) {    //bepalen welke digit aanpassen
          case 1:                //add to lsb
            if (add == 1) {
              Kset = Kset + 1;
            }
            else if (sub == 1) {
              Kset = Kset - 1;
            }
            else {
              break;
            }
          case 2:                 // add to 2nd bit
            if (add == 1) {
              Kset = Kset + 10;
            }
            else if (sub == 1) {
              Kset = Kset - 10;
            }
            else {
              break;
            }
          case 3:
            if (add == 1) {
              Kset = Kset+100;
            }
            else if (sub == 1) {
              Kset = Kset-100;
            }
            if (Kset > 360) {
              Kset = Kset - 360;
            }
            if (Kset < 0) {
              Kset = 360 + Kset;
            }
            break;
        }
        if (Kset > 360) {
          Kset = Kset - 360;
          lastShft = 0;
        }
      }
    } while (ent = !HIGH); //stop by enter
    if (shft == 1 && lastShft == 0) {
      lastShft = 1;
    }
  }
  R = Kset - Kact;
  fuzzy->setInput(1, R);
  fuzzy->setInput(2, r);
  digitalWrite(ledG, 255);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("act course: ");
  lcd.print(Kact);
  lcd.setCursor(0, 1);
  lcd.print("set course: ");
  lcd.print(Kset);
  fuzzy->fuzzify();
  float deltac = fuzzy->defuzzify(1);
  deltac = 90 + deltac;
  do {
    delta = Rudder.read();
    while (delta != deltac) {
      Rudder.slowmove(delta, 3);
    }
  }
  while (delta = encode);
  if (delta != encode) {
    lcd.setCursor(0, 1);
    lcd.print("rudder error");
    digitalWrite(ledR, HIGH);
  }
}
