#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <EEPROM.h> //what is this even for...
#include <MCP4725.h> //i hate this one!!
#define fourthByte(w) ((uint8_t)((w) >> 24))
#define thirdByte(w) ((uint8_t)((w) >> 16))
#define secondByte(w) ((uint8_t)((w) >> 8))
#define firstByte(w) ((uint8_t)((w)&0xFF))
LiquidCrystal_I2C lcd(0x27,20,4);
MCP4725 BrakeDAC(0x61);
MCP4725 ThrottleDAC(0x60);

/* TwoWire i2cData = TwoWire(0);
TwoWire i2cControl = TwoWire(1); */

#define _FALLING 0
#define _RISING 1

bool boot_check = false, CCON = false, brake_check = false, CCSET = false;

byte contrl[9];
byte tele[20];
uint16_t PedV = 6;

float motorW_float = 1, solarW1_float = 4, solarW2_float = 3, BattA_float = 0.4, BattV_float = 5, BattW = 2, CCV = 0, CCVmot = 0;
long motorW_int = 0, solarW1_int = 0, solarW2_int = 0, BattA_int = 0, BattV_int = 5, CCV_int = 0;

//inpu6t pins
const unsigned int 
leftTurnButton = 39,
rightTurnButton = 41,
hazardButton = 32,
daylightButton = 30,
CCButton = 37,
CCUpButton = 35,
CCDownButton = 34,
HVButton = 28,
PV1Button = 26,
PV2Button = 24,
PVOffButton = 8,
throttleInput = 15,
HVOnCo = 9,
brakeAn = 14,
hall_pulse_pin = 11, //mtr dgry (meter dark gray) pin??
brakeOn = 10,
CConButton = 21,

//output pins
brakeOut = 5, //brake
leftOut = 3, //left
daylightOut = 2, //drl
rightOut = 4, //right turn
PV1Out = 27,
PV2Out = 25,
softStartOut = 6,
HighVOut = 7,
HVLed = 29,
statusLed = 13, //led built in
hazardLEDOut = 33,
drlLEDOut = 31,
leftLEDOut = 38,
rightLEDOut = 40,
CConLed = 20,
CCLed = 36; //also for throttle limiter

//previous activates for debounce
unsigned long lastLTurn = 0,
lastRTurn = 0,
lastHazard = 0,
lastDaylight = 0,
lastTLOn = 0,
lastTLUp = 0,
lastTLDown = 0,
lastAllPV = 0,

//lcd debounce times
prevUpd = 0,
prevSpdUpd = 0,
prevLcdUpd = 0,
prevCConUpd = 0,

prevBrakeUpd = 0,
prevPowerUpd = 1000,
prevPVUpd = 5000,
prevBattVUpd = 5000,

//blinky
diff = 0,
LEDLastOn = 0,
LightLastOn = 0,

//power debounces
lastHVToggle = 0,
lastPV1On = 0,
lastPV2On = 0,
lastSoftStart = 0,
//lastPVOff = 0;

//odometer things
odo_run = millis(),
net_run = millis(),
dat_run = millis(),
start = millis(),

//hall & speed vars
hall_now = 0,
hall_last = 0, //
LTime = 0,
HTime = 0,
TTime = 0, //
hallpulses = 0, //
total_distance = 0,
total_distance_read = 0,
trip_distance = 0,
kph = 0,  		   //what for though??
hall_factor = 35685, 	   // 1586/16 * 3.6

//power buffer
buffer=3000;

//throttle limiter logic
float TLv = 0,   	   //maximum voltage going to motor controller under throttle limiter constraints
actualTLv = 0,
scaledInPedalV = 0,
outPedalV = 6;
int inPedalV = 0,
PedalV = 0;
float prevPedalV = 0;
int outThrot = 0,

//timing for diagnostic
prevLoopTime = 0,
prevSendTime = 0,

//analog brake input\output
outBrake = 0;
float Pulses_PerMS = 0.0,
//HallKPH = 0.0,
HallKPH = 0.0;
uint8_t total_distance_arr[4] = {};
float WheelCircMM = 1735.0,
timeout = /*60000*/85000;

//states
bool turningL = false,
turningR = false,
hazardOn = false,
daylightOn = false,
TLOn = false,
TLOnPrev = false,
Coasting = false,
TLup = false, //dont need?
TLdown = false, //dont need?
HVOn = false,
PV1On = false,
PV2On = false,
PVOff = false,
onSequ = false,
offSequ = false,
//softStartOn = false; //maybe???
PVcanEnable = true,
brakingNow = false,
HVOnInProgress=false,
//lastHallLcdReset = false;
canCoast = false;

//floats
float float32min=1.17549435082e-38,
float32max=3.40282346639e+38;

//button stuff
unsigned long buttonsN=0,
buttonPins[20]={0},
conditions[20]={0},
lastRead[20]={0},
heldButtonPins[5] = {0},//
initialPressTime[5]={0},//
intervals[5]={0},//
heldConditions[5]={0},//
heldLastRead[5] = {0},//
heldButtonsN=0;//
//heldButtonsOffset=0;//
void (*functions[20])(unsigned long)={0};
void (*heldFunctions[5])(unsigned long)={0};//

//temporary
// bool test = false;





//setup, other stuff
void setup() {
  //input
  pinMode(leftTurnButton, INPUT_PULLUP);
  pinMode(rightTurnButton, INPUT_PULLUP);
  pinMode(hazardButton, INPUT_PULLUP);
  pinMode(daylightButton, INPUT_PULLUP);
  pinMode(CCButton, INPUT_PULLUP);
  pinMode(CCUpButton, INPUT_PULLUP);
  pinMode(CCDownButton, INPUT_PULLUP);
  pinMode(HVButton, INPUT_PULLUP);
  pinMode(PV1Button, INPUT_PULLUP);
  pinMode(PV2Button, INPUT_PULLUP);
  pinMode(PVOffButton, INPUT_PULLUP);
  pinMode(CConButton, INPUT_PULLUP);
  //pinMode(softStartButton, INPUT_PULLUP);
  pinMode(HVOnCo, INPUT_PULLUP);
  pinMode(brakeAn, INPUT);
  pinMode(throttleInput, INPUT);
  pinMode(brakeOn, INPUT);
  pinMode(hall_pulse_pin, INPUT);

  //output
  pinMode(daylightOut, OUTPUT);
  pinMode(rightOut, OUTPUT);
  pinMode(leftOut, OUTPUT);
  pinMode(PV1Out, OUTPUT);
  pinMode(PV2Out, OUTPUT);
  pinMode(softStartOut, OUTPUT);
  pinMode(HighVOut, OUTPUT);
  pinMode(CCLed, OUTPUT);
  pinMode(CConLed, OUTPUT);
  pinMode(statusLed, OUTPUT);
  pinMode(HVLed, OUTPUT);
  pinMode(hazardLEDOut, OUTPUT);
  pinMode(drlLEDOut, OUTPUT);
  pinMode(leftLEDOut, OUTPUT);
  pinMode(rightLEDOut, OUTPUT);
  pinMode(brakeOut, OUTPUT);
 
  ThrottleDAC.begin();
  BrakeDAC.begin();

//  attachInterrupt(digitalPinToInterrupt(hall_pulse_pin), pulse, FALLING);

  AssignButton(leftTurnButton, TurnLeft, _FALLING);
  AssignButton(rightTurnButton, TurnRight, _FALLING);
  AssignButton(hazardButton, Hazard, _FALLING);
  AssignButton(daylightButton, Daylight, _FALLING);
  AssignButton(CCButton, CC, _FALLING);
  AssignButton(CCUpButton, CCVUp, _FALLING);
  AssignButton(CCDownButton, CCVDown, _FALLING);
  AssignButton(CConButton, CCon, _FALLING);
  AssignButton(HVButton, HighVoltageToggle, _FALLING);
  AssignButton(PV1Button, SolarPanel1, _FALLING);
  AssignButton(PV2Button, SolarPanel2, _FALLING);
  //assignHeldButton(TLButton, Coaster, _FALLING, 1000);
  //AssignButton(PVOffButton, SolarPanelOff, _FALLING);
  // AssignButton(digitalPinToInterrupt(softStartButton), SoftStart, _FALLING);

  //library/system setup
  Serial.begin(9600);
  Serial4.begin(19200);
  //Serial5.begin(9600); //data teensy
  Wire.begin(0x04);
  Wire.setSDA(18);
  Wire.setSCL(19); //MY STUPID BRAIN SET TWO SDAS ATSVGHGSDBUXHNDJIUJHBNHSIJ
  //odometer_read(); //needed??
  //Wire1.onRequest(currDataTransmit);

  //start values
  Serial.println("Program not dead yet.");
  //digitalWrite(statusLed, HIGH);
 
  //lcd setup
  lcd.init(); // IMPORTANT: can also use to refresh lcd if it desyncs
  lcd.clear();
  //labels left
    lcdLabel("MOTW", 0, 0);
    lcdLabel("BATW", 0, 1);
    lcdLabel("PV2", 0, 2);
    lcdLabel("PV1", 0, 3);

    lcdLabel("0", 8, 0);
    lcdLabel("0", 8, 1);
    lcdLabel("0", 8, 2);
    lcdLabel("0", 8, 3);
    //labels right
    lcdLabel("BATV", 10, 0);
    lcdLabel("PEDV", 10, 1);
    lcdLabel("CCV", 11, 2);
    lcdLabel("MPH", 11, 3); //might be mph later tho

    lcdLabel("%", 19, 1);
    lcdLabel("%", 19, 2);

    lcdLabel("0", 19, 0);
    lcdLabel("0", 18, 1);
    lcdLabel("0", 18, 2);
    lcdLabel("0", 19, 3);
  lcd.backlight();
  digitalWrite(brakeOut, HIGH);
  digitalWrite(rightOut, HIGH);
  digitalWrite(leftOut, HIGH);
  i2cSender(&ThrottleDAC, 500);
}

//button assignment
unsigned long AssignButton(const unsigned int pin, void(*f)(unsigned long),const unsigned int condition)
{
  buttonPins[buttonsN]=pin;
  conditions[buttonsN]=condition;
  functions[buttonsN]=f;
  lastRead[buttonsN]=digitalRead(pin);
  buttonsN++;
  return buttonsN-1;
}
void Buttons()
{
  for (unsigned int i=0;i<buttonsN;i++)
  {
  //Serial.print("Current:");Serial.print(buttonPins[i]);Serial.println(digitalRead(buttonPins[i]));
  //Serial.print("Before: "); Serial.println(lastRead[i]);
  if (digitalRead(buttonPins[i])==conditions[i] && lastRead[i]!=conditions[i])
  {
    (*functions[i])(i);
  }
  lastRead[i]=digitalRead(buttonPins[i]);
  }
}
void heldButtons() {
  for (unsigned int i=0;i<buttonsN;i++)
  {
      //Serial.print("Current:");Serial.print(buttonPins[i]);Serial.println(digitalRead(buttonPins[i]));
      //Serial.print("Before: "); Serial.println(lastRead[i]);
      if ((digitalRead(heldButtonPins[i]) == heldConditions[i]) && (heldLastRead[i] != heldConditions[i])) {
     	 initialPressTime[i] = millis();
     	 heldLastRead[i] = digitalRead(heldButtonPins[i]);
      }
	 
      if (debounce(millis(), initialPressTime[i], intervals[i]) && digitalRead(heldButtonPins[i]) == heldLastRead[i]) {
        (*functions[i])(i);
      }
      //lastRead[i]=digitalRead(buttonPins[i]);
  }
}

void assignHeldButton(const unsigned int pin, void(*f)(), const unsigned int cond, const unsigned long interval) {

    
    heldButtonPins[heldButtonsN] = pin;
    initialPressTime[heldButtonsN] = millis();
    intervals[heldButtonsN] = interval;
    heldFunctions[heldButtonsN] = f;
    heldConditions[heldButtonsN] = cond;
    heldLastRead[heldButtonsN]=digitalRead(pin);
    heldButtonsN++;
}

//logic debounce
bool debounce(const unsigned long Time, const unsigned long Ptime, const int interval) {
  //checks to see if the time last logged and the current time is greater or equal to the interval
  if (Time - Ptime >= interval) {
  //if that is true, it returns true
  return true;
  } else return false; //else it returns false
}

//lcd stuff
void lcdLabel(char string[], int cursorX, int cursorY) {
  lcd.setCursor(0,0);
  //completely unnecessarily complicated print method
  int length = String(string).length(); //grabs length of label
  lcd.setCursor(cursorX, cursorY); //sets to line
  for(int i = 0; i < length; i++) {
  //prints each letter of label to ensure there are now discrepancies in letter placement
  lcd.print(string[i]);
  }
}
void lcdDataWriter(float data, int space, int cursorX, int cursorY, int emptyspace) {
  lcd.setCursor(0,0);
  int length = String(data).length();
  lcd.setCursor(cursorX, cursorY); //grabs length of data
  //clears data line with specified fill type
  lcd.rightToLeft();
  for (int i = space + emptyspace; i > 0; i--){ lcd.print(" ");}
  //fills data line with data value
  lcd.leftToRight();
  //if (type == "float") {
  //if data is a decimal, it prints in this manner
  /* if (spaces > 0) {
	lcd.setCursor(cursorX - length + 2 + spaces, cursorY);
	lcd.print(" ");
  } */
  lcd.setCursor(cursorX - length + 2, cursorY);
  lcd.print(data, 1); //number of values after the point is 1
  //}

}
void lcdDataWriter(int data, int space, int cursorX, int cursorY) {
  lcd.setCursor(0,0);
  int length = String(data).length();
  lcd.setCursor(cursorX, cursorY);
  lcd.rightToLeft();
  for (int i = space; i > 0; i--) lcd.print(" ");
  lcd.leftToRight();
  lcd.setCursor(cursorX - length + 2, cursorY);
  lcd.print(data, 10); //number of values after the point is 1
}
void lcdDataWriter(float data, unsigned int divider, int space, int cursorX, int cursorY, int emptyspace) { // do i even need this? //divider denotes decimal point location btw
  //was stupid and tried to do this the really complicated way..
  //empty space is extended space between actual expected number and label
  float holder = data/divider;
  space += 1;
  lcd.setCursor(0,0);
  int length = String(data).length();
  lcd.setCursor(cursorX-1, cursorY); //grabs length of data
  //clears data line with specified fill type
  lcd.rightToLeft();
  for (int i = space + emptyspace - 1; i > 0; i--) lcd.print(" ");
  //fills data line with data value
  lcd.leftToRight();
  lcd.setCursor(cursorX - length + 2, cursorY);
  lcd.print(holder, 1); //number of values after the point is 1
}

//good i2c sending
void i2cSender(MCP4725* MCP, unsigned long val) { //could set as bool if really wanted to, except it isnt really returning any trues
  MCP->setValue(val);
}

//bad i2c sending
float frabs(float x)
{
  return x>0?x:-x;
}
void sendFloat32(float x)
{
  unsigned int signBIT=0;
  if (0.0>x)
  {
      signBIT=1;
  }
  int exp=0;
  unsigned long frbin=LARSEN___________a___frbinexpwithbias(frabs(x),&exp);
  unsigned long bits=signBIT<<31 | (exp&0xFF) << 23 | frbin&0b11111111111111111111111;
    //Serial.printf("\nfrbits: %lx\n",frbin);
    //Serial.printf("\nbits: %lx\n",bits);
  Wire.write((unsigned char)(bits&0xFF));
  Wire.write((unsigned char)(bits>>8)&0xFF);
  Wire.write(((unsigned char)(bits>>16)&0xFF));
  Wire.write(((unsigned char)(bits>>24)&0xFF));
}
void sendUInt8(unsigned long x)
{
  Wire.write((unsigned char)(x&0xFF));
}
void sendUInt16(unsigned long x)
{
  Wire.write((unsigned char)x&0xFF);
  Wire.write((unsigned char)x>>8&0xFF);
}
void sendUInt32(unsigned long x)
{
  Wire.write((unsigned char)x&0xFF);
  Wire.write((unsigned char)x>>8&0xFF);
  Wire.write((unsigned char)x>>16&0xFF);
  Wire.write((unsigned char)x>>24&0xFF);
}
/*void sendInt8(int x)
{
  unsigned int mag=abs(x)&0x7F;
  unsigned int int8bin=mag;
  unsigned int sign=0;
  if (0>x){
      sign=1;
      int8bin=~mag+1|sign<<7; // check precedence
  }
  Wire1.write((unsigned char)int8bin&0xFF);
}*/
void sendInt16(int x)
{
  unsigned int mag=abs(x)&0x7FFF;
  unsigned int int16bin=mag;
  unsigned int sign=0;
  if (0>x){
      sign=1;
      int16bin=~mag+1|sign<<15;
  }
  Wire.write((unsigned char)int16bin&0xFF);
  Wire.write((unsigned char)int16bin>>8&0xFF);
}
void sendInt32(long x)
{
  unsigned long mag=abs(x)&0x7FFFFFFF;
  unsigned long int32bin=mag;
  unsigned int sign=0;
  if (0>x){
      sign=1;
      int32bin=~mag+1|sign<<31;
  }
  Wire.write((unsigned char)int32bin&0xFF);
  Wire.write((unsigned char)int32bin>>8&0xFF);
  Wire.write((unsigned char)int32bin>>16&0xFF);
  Wire.write((unsigned char)int32bin>>24&0xFF);
}
unsigned long LARSEN___________a___frbinexpwithbias(float x, int* exp)
{
  unsigned long bin=0;
  unsigned int i=0;
  float normalized=x;
    
 
  if (x==0.0 || float32min>x) { // 0.0 or less than min
      return 0;
  } else if (float32max<x) { // larger than max (inf)
      *exp=0b11111111;
      return 0;
  }
 
  if (x>=2.0)
  {
      while (normalized>=2.0) // normalize
      {
   	   normalized/=2;
   	   (*exp)++;
      }
  } else if (x<1.0) {
      while (normalized<1.0) // normalize
      {
   	   normalized*=2;
   	   (*exp)--;
      }
  } else if (x==1.0) { // weird
      *exp=127;
      return 0;
  }

  float temp=normalized-1.0;
  while (true)
  {
      temp*=2.0;
      if (temp>1.0)
      {
   	   bin|=1<<(22-i);
   	   temp-=1.0;
      }
      if (temp==1.0)
      {
   	bin|=1<<(22-i);
   	   break;
      }
      i++;
 	if (i==23)
 	{
   	break;
 	}
  }
  (*exp)+=127;
  return bin;
}

unsigned long how_long(const char* s) {
  unsigned long n = millis();
  unsigned long d = n-start;
  Serial.printf("%s time-> %d\n",s,d);
  start = n;
  return d;
}

//lights
void blinkLight(unsigned long output) {
  diff=millis()-LEDLastOn;
  if (diff>500 && diff<1000)
  {
    digitalWrite(output,LOW);
  } else if (diff>1000) {
    digitalWrite(output,HIGH);
    LEDLastOn=millis();
  }
}
void blinkOutLight(unsigned long output1, unsigned long output2) {
  diff=millis()-LightLastOn;
  if (diff>500 && diff<1000)
  {
    digitalWrite(output1,LOW);
    digitalWrite(output2,LOW);
  } else if (diff>1000) {
    digitalWrite(output1,HIGH);
    digitalWrite(output2,HIGH);
    LightLastOn=millis();
  }
}
void blinkOutLight(unsigned long output1) {
  diff=millis()-LightLastOn;
  if (diff>500 && diff<1000)
  {
    digitalWrite(output1,LOW);
  } else if (diff>1000) {
    digitalWrite(output1,HIGH);
    LightLastOn=millis();
  }
}
uint32_t PedalV_int;
//main
void loop() {
  

    //I dont know how this works, dont mess with it
  if(Serial4.available() >= 20){
    if(!boot_check){
      boot_check = true;
      Serial4.end();
      Serial4.begin(19200);
    }
    Serial4.readBytes(tele, 20);
    delay(1);
    Serial4.flush();
    
    motorW_int = (tele[0] << 24) + (tele[1] << 16) + (tele[2] << 8) + tele[3];
    memcpy(&motorW_float, &motorW_int, 4);
    motorW_float = -(motorW_float);

    solarW1_int = (tele[4] << 24) + (tele[5] << 16) + (tele[6] << 8) + tele[7];
    memcpy(&solarW1_float, &solarW1_int, 4);
    //solarW1_float = -(solarW1_float);

    solarW2_int = (tele[8] << 24) + (tele[9] << 16) + (tele[10] << 8) + tele[11];
    memcpy(&solarW2_float, &solarW2_int, 4);
    //solarW2_float = -(solarW2_float);

    BattV_int = (tele[12] << 24) + (tele[13] << 16) + (tele[14] << 8) + tele[15];
    memcpy(&BattV_float, &BattV_int, 4);

    BattA_int = (tele[16] << 24) + (tele[17] << 16) + (tele[18] << 8) + tele[19];
    memcpy(&BattA_float, &BattA_int, 4);



    BattW = -(BattA_float*BattV_float);
    lcdPowerUpdate();
    
  }   
  if(Serial4.availableForWrite() >= 9){
    memcpy(&PedalV_int, &outPedalV, 4);
    contrl[0] = fourthByte(PedalV_int);
    contrl[1] = thirdByte(PedalV_int);
    contrl[2] = secondByte(PedalV_int);
    contrl[3] = firstByte(PedalV_int);

    memcpy(&CCV_int, &CCV, 4);
    contrl[4] = fourthByte(CCV_int);
    contrl[5] = thirdByte(CCV_int);
    contrl[6] = secondByte(CCV_int);
    contrl[7] = firstByte(CCV_int);

    contrl[8] = brakingNow + (CCON << 1) + (CCSET << 2) + (HVOn << 3) + (daylightOn << 4) + (hazardOn << 5) + (PV1On << 6) + (PV2On << 7);

    Serial4.write(contrl, 9);
    delay(1);
    Serial4.flush();
    
  }
  // ^                              ^
  // | Dont touch this, it'll break |
  // |                              |

  lcdDisplayUpdate();
  
  /*------temporary------*/
  digitalWrite(statusLed, LOW);

 
  /*------inputs------*/
  Buttons();							   // sequential button inputs rather than prioritized interrupts
  //heldButtons();
  //how_long("Buttons");

  /*------lights------*/
  Lights();      					   // lights code
  //BrakeLight(); 					   // brake light specific
  //how_long("Lights");

  /*------throttle limiter, pedals & brake------*/
  BrakeGeneral();					   // analog brake (maybe interrupt???)
  //how_long("brakes");
  pedalInToOut();					   // basically makes any press on pedal ramp up slowly, and any release an immediate fall
  //how_long("intooutped");
  outPedCC();    					   // takes into account if throttle limiter is on or not, maps to analog output
  //how_long("cc");
  GetHallKPH();  					   // gets current kph (maybe move???)
  //how_long("speed");
  //CCLight(); 						   // shows current cc/tl status
  //how_long("turnccled");

  /*------power------*/
  HighVoltageMain(); 				   // for turning on\off high voltage
  HVLedOn();  						   // turns light stuff statuses
  //how_long("HV");


  /*------i2c sendings------*/
  i2cSender(&ThrottleDAC, outThrot);  	// sends out the current throttle voltage
  //how_long("i2c sender");
 
  /*------solar panels------*/
  SolarPanelOff();
  //how_long("solar panels");
 
 
  /*------data------*/
  //odo_run = run_every(30000,odo_run,odometer_write);
  digitalWrite(statusLed, LOW);
  
  diagnosticRun(); 					   // diagnostic monitor for monitoring
  //currDataTransmit();
  //how_long("diagnositcs");
}


long lastlcd = 0;

//display shenanigans
void lcdPowerUpdate(){
  if (debounce(millis(), prevBattVUpd, 5000)) {
    lcdDataWriter(BattV_float, 0, 19, 4, 0);
    prevBattVUpd = millis();
  }

  if (debounce(millis(), prevPowerUpd, 1000)) {
    lcdDataWriter(round(BattW), 4, 7, 1);
    lcdDataWriter(round(motorW_float), 4, 7, 0);
    prevPowerUpd = millis();
  }

  if (PV2On | PV1On && debounce(millis(), prevPVUpd, 2000)) {
    lcdDataWriter(round(solarW1_float), 5, 7, 3);
    lcdDataWriter(round(solarW2_float), 5, 7, 2);
    prevPVUpd = millis();
  }

  if(debounce(millis(), lastlcd, 10000)){
    lcd.clear();
    lcdLabel("MOTW", 0, 0);
    lcdLabel("BATW", 0, 1);
    lcdLabel("PV2", 0, 2);
    lcdLabel("PV1", 0, 3);

    lcdLabel("0", 8, 0);
    lcdLabel("0", 8, 1);
    lcdLabel("0", 8, 2);
    lcdLabel("0", 8, 3);
    //labels right
    lcdLabel("BATV", 10, 0);
    lcdLabel("PEDV", 10, 1);
    lcdLabel("CCV", 11, 2);
    lcdLabel("MPH", 11, 3); //might be mph later tho

    lcdLabel("%", 19, 1);
    lcdLabel("%", 19, 2);

    lcdLabel("0", 19, 0);
    lcdLabel("0", 18, 1);
    lcdLabel("0", 18, 2);
    lcdLabel("0", 19, 3);
    lcdDataWriter(int(20*CCV), 3, 17, 2);
    prevPowerUpd = 1000;
    prevPVUpd = 5000;
    prevBattVUpd = 5000;
    lastlcd = millis();
  }
  

}

void lcdDisplayUpdate() {
  
    
	if (debounce(millis(), prevSpdUpd, 200)) { //speed
    lcdDataWriter(HallKPH*0.621, 3, 19, 3, 3);
 	  prevSpdUpd = millis();
  }


}

//diagnostic runner monitor thing
void diagnosticRun() {
  Serial.println("[V]------------diagnosticMonitor------------[V]");
  Serial.print("lastLcdReSync (ms) -> "); Serial.println(prevLcdUpd);
  Serial.print("lastDataDump (ms) -> "); Serial.println(prevSendTime);
  Serial.println("[-HV-]");
  if (HVOn) Serial.println("HV system active."); else Serial.println("HV system not active.");
  if (HVOnInProgress) Serial.println("HV on sequence activated."); else Serial.println("HV on sequence deactivated.");
  if (digitalRead(softStartOut) == HIGH) Serial.println("Soft start relay enabled."); else Serial.println("Soft start relay disabled.");
  if (digitalRead(HighVOut) == HIGH) Serial.println("HV relay enabled."); else Serial.println("HV relay disabled.");
  if (digitalRead(HVOnCo) == LOW) Serial.println("HV relay enable confirmed."); else Serial.println("HV relay enable not confirmed or is N/A.");
  Serial.print("[-CC-]"); Serial.println("");
  Serial.print("Input Readings:"); Serial.println("");
  Serial.print("    Cruise Control Button Status: "); Serial.println(digitalRead(CCButton));
  Serial.print("    Cruise Control Up Button Status: "); Serial.println(digitalRead(CCUpButton));
  Serial.print("    Cruise Control Down Button Status: "); Serial.println(digitalRead(CCDownButton));
if (TLOn) Serial.println("Cruise control activated."); else Serial.println("Cruise control deactivated.");
  Serial.print("Internal Statuses:"); Serial.println("");
  Serial.print("    Cruise control voltage (1023 to 0) -> "); Serial.println(CCVmot);
  Serial.print("    Cruise control voltage out (5 to 0) -> "); Serial.println(CCV);
  Serial.print("    Coast Status -> "); Serial.println(CCSET);
  Serial.println("[-AN-]");
  Serial.print("anBrakeCurrOut (4095 to 0) -> "); Serial.println(outBrake);
  Serial.print("anBrakeCurrIn (1023 to 0) -> "); Serial.println(analogRead(brakeAn));
  Serial.println("[-PD-]");
  Serial.print("PedalInUnscaled (1023 to 0) -> "); Serial.println(analogRead(throttleInput));
  Serial.print("PedalOutUnscaled (1023 to 0) -> "); Serial.println(outPedalV);
  Serial.print("PedalOutScaled (w/ cc) (4095 to 0) -> "); Serial.println(outThrot);
  //Serial.print("testVal -> "); Serial.println(test); //temporary
  Serial.println("[-BR-]");
  //Serial.print("a -> "); Serial.println(outBrake);
  Serial.print("overallBrakeStatus -> "); Serial.println(brakingNow);
  Serial.print("PedBrakeStatus -> "); Serial.println(digitalRead(brakeOn));
  Serial.println("[-SP-]");
  Serial.print("rawKPH -> "); Serial.println(HallKPH);
  //Serial.print("kilometersThisTrip -> "); Serial.println(HallKPH);
  Serial.println("[-LI-]");
  Serial.print("rightTurnIn -> "); Serial.println(digitalRead(rightTurnButton));
  Serial.print("leftTurnIn -> "); Serial.println(digitalRead(leftTurnButton));
  Serial.print("HazardState -> "); Serial.println(digitalRead(hazardOn));
  Serial.print("HazardIn -> "); Serial.println(digitalRead(hazardButton));
  Serial.print("daylightOut -> "); Serial.println(digitalRead(daylightOut));
  Serial.print("rightOut -> "); Serial.println(digitalRead(rightOut));
  Serial.print("leftOut -> "); Serial.println(digitalRead(leftOut));
  Serial.print("brakeOut -> "); Serial.println(digitalRead(brakeOut));
  Serial.print("MotorPower -> "); Serial.println(motorW_float);
  Serial.print("BatteryPower -> "); Serial.println(BattW);
  Serial.print("Solar2Power -> "); Serial.println(solarW2_float);
  Serial.print("Solar1Power -> "); Serial.println(solarW1_float);
  Serial.print("BatteryVoltage -> "); Serial.println(BattV_float);
  Serial.print("TelemetryArray -> "); 
  for(int i = 0; i < 20;){
    Serial.print(tele[i]);
    Serial.print("  ");
    i++; 
  }
  Serial.println();
  Serial.print("ControlArray -> ");
  for(int i = 0; i < 9;){
    Serial.print(contrl[i]);
    Serial.print("  ");
    i++; 
  }
  Serial.println();
  Serial.print("timeSinceLastLoop (ms) -> "); Serial.println((millis() - prevLoopTime));
  Serial.println();
  Serial.println("[^]------------diagnosticMonitor------------[^]");
 
  prevLoopTime = millis();
 
}

//speed, hall sensors, & distance-ish
/* unsigned long lastSpdAdd = 0;
unsigned int numAdds = 0;
float kphHolder = 0; */
void GetHallKPH() { //note to self: make this start registering after 10 kph
//Hall pin is on the other teensy (aka main), we can either move this code here (probably harder) or just get the data from here to the other teensy
 
  hall_now = millis();   //for distance!!!!

  //gets current speed in kph
  LTime = pulseIn(hall_pulse_pin, LOW, timeout);
  HTime = pulseIn(hall_pulse_pin, HIGH, timeout);
  TTime = LTime+HTime;
  //total_distance = total_distance_read + trip_distance; //odometer only!!!!!
  float WFreq = 1.0/TTime;
  //Log(&Serial,0,"TTime",TTime);     //do i need to log?? not really sure
  //Log(&Serial,0,"WFreq",WFreq);
  hall_last = hall_now;
  //per one pulse
  HallKPH = (float)TTime>0.0 ? WFreq*WheelCircMM*225.0:0.0;

}

//lights (WIP - Lexi)
void TurnLeft(unsigned long i) { //NOT toggle; momentary...
  if (debounce(millis(), lastLTurn, 100)) {
  turningL = true;
  lastLTurn = millis();
  }
}
void TurnRight(unsigned long i) { //NOT toggle; momentary...
  if (debounce(millis(), lastRTurn, 100)) {
  turningR = true;
  lastRTurn = millis();
  }
}
void Hazard(unsigned long i) { //toggle
  if (debounce(millis(), lastHazard, 100)) {
    hazardOn = !hazardOn;

    lastHazard = millis();
  }
}
void Daylight(unsigned long i) { //toggle
  if (debounce(millis(), lastDaylight, 100)) {
    daylightOn = !daylightOn;
    lastDaylight = millis();
  }
}
void Lights() {
  unsigned long now=millis();
  if (turningL) {
      if ((now-lastLTurn)<=5000) {
        //Serial.println("Left turn lights flashing now");
        // flash lights for left turn
        blinkLight(leftLEDOut);
        blinkOutLight(leftOut); //WHY DONT YOU WORK!!!!!!
      } else {
        // Serial.println("STOP Left turn lights");
        // STOP flash lights for left turn
        digitalWrite(leftLEDOut, LOW);
        digitalWrite(leftOut, HIGH);
        turningL = false;
      }
  }
  if (turningR) {
      if ((now-lastRTurn)<=5000) {
        //Serial.println("Right turn lights flashing now");
        // flash lights for right turn
        blinkLight(rightLEDOut);
        blinkOutLight(rightOut); //you also dont work!!!!!!
        //digitalWrite(rightOut, RIGHT);
      } else{
        // Serial.println("STOP Right turn lights");
        // STOP flash lights for right turn
        digitalWrite(rightLEDOut, LOW);
        digitalWrite(rightOut, HIGH);
        turningR = false;
      }
  }
  if (hazardOn) { // flipped?
    //on state code here
    blinkLight(hazardLEDOut);
    blinkOutLight(leftOut, rightOut);
    //blinkOutLight(rightOut);
  } else if (!hazardOn) {
    //off state code here
    // STOP flash lights for both left and right
    digitalWrite(hazardLEDOut, LOW);
    digitalWrite(rightOut, HIGH);
    digitalWrite(leftOut, HIGH); //left just, doesn't work here
  }
	digitalWrite(drlLEDOut, daylightOn);
	digitalWrite(daylightOut, !daylightOn);
}
void BrakeLight() {
    if (brakingNow) digitalWrite(brakeOut, LOW); else digitalWrite(brakeOut, HIGH);
    //Serial.println("brakelight 1080");
}





unsigned long prevCCUpUpd = 0, prevCCDownUpd = 0, prevCCUpd = 0;
bool CCstatus = true;
float lastCCV = 0;

void CCon(unsigned long i) {
  if(debounce(millis(), prevCConUpd, 200)){
    CCON = !CCON;
    CCstatus = true;
    delay(1);
    if(!CCON){
      if(CCSET){
        CCSET = false;
      }
      CCV = 0;
      CCVmot = 0;
    }
    lcdDataWriter(int(20*CCV), 3, 17, 2);
    digitalWrite(CConLed, CCON);
    digitalWrite(CCLed, CCSET);
    prevCConUpd = millis();
  }
}

void CC(unsigned long i) {
  if(CCON && debounce(millis(), prevCCUpd, 200) && !brakingNow){
    CCSET = !CCSET;
    prevCCUpd = millis();
    if(CCSET && outPedalV > 0){
      CCVmot = outPedalV;
      CCV = map(outPedalV, 0, 1023, 0, 5);
    }
    if(CCSET){
      Coasting = false;
    }
    lcdDataWriter(int(20*CCV), 3, 17, 2);
    digitalWrite(CCLed, CCSET);
  }
}


//throttle limiter, actual throttle & analog brake
void CCVUp(unsigned long i) { //toggle for enable
  if(CCON && debounce(millis(), prevCCUpUpd, 50) && !brakingNow) {
    CCV = min(5, CCV+0.1);
    CCstatus = true;
    prevCCUpUpd = millis();
    CCVmot = map(CCV, 0, 5, 0, 1023);
    lcdDataWriter(int(20*CCV), 3, 17, 2);
  }
}

void CCVDown(unsigned long i) {
  if(CCON && debounce(millis(), prevCCDownUpd, 50) && !brakingNow) {
    if(CCV > 0){
      CCV = min(5, CCV-0.1);
    }
    CCstatus = true;
    prevCCDownUpd = millis();
    CCVmot = map(CCV, 0, 5, 0, 1023);
    lcdDataWriter(int(20*CCV), 3, 17, 2);
  }
}

unsigned long lastThrottleUp=0;
void pedalInToOut() {
  outPedalV = min(1023, max(0 ,map(analogRead(throttleInput), 175, 800, 0, 1023)));
  if (debounce(millis(), prevUpd, 200)) { //pedal
  	lcdDataWriter(int(map(outPedalV, 0, 1023, 0, 100)), 3, 17, 1); //ped v

 	  prevUpd = millis();
  }
}

void outPedCC() { 
  unsigned long vHolder = 0;
  if (CCSET && !Coasting) {
    vHolder = CCVmot;
  } else {
    vHolder = outPedalV;
  }
  outThrot = min(4095, map(vHolder, 0, 1023, 0, 4095));
}
void BrakeGeneral() {
    /*
    rewrite brakes so they work on a single if function
    */
    
  //should disable throttle input and apply brakes in accordance to potentiometer input.
  //if larger than 0.8 volt, turn off throttle and scale signal to the 5 volt range.
  unsigned long inBrake = analogRead(brakeAn);
  //unsigned long pedInBrake = map(analogRead(brakeOn), 0, 1023, 0, 3.3);
  //bool analogBrakingNow = false;
  if (digitalRead(brakeOn) || (inBrake > 230)) {
    if (CCON) Coasting = true;
    brakingNow = true;
    CCSET = false;
    CCstatus = false;
    digitalWrite(CCLed, CCSET);
    digitalWrite(brakeOut, LOW);
    //if (digitalRead(brakeOn)) i2cSender(&BrakeDAC, 1250);

    if (inBrake > 230) i2cSender(&BrakeDAC, outBrake);
  } else {
    brakingNow = false;
    digitalWrite(brakeOut, HIGH);
    i2cSender(&BrakeDAC, 100);
  }
 
 

  outBrake = min(max(floor(map(inBrake, 179, 861, 0, 4095)), 0), 4095);
  // Serial.print("anBrakeCurr "); Serial.println(outBrake);
  // Serial.print("anBrakeCurrIn "); Serial.println(inBrake);
}

//power
void HVLedOn() {
  if (HVOn || digitalRead(HVOnCo) == LOW)
  {
  digitalWrite(HVLed, HIGH);
  return;
  }
  if (HVOnInProgress)
  {
  blinkLight(HVLed);
  } else digitalWrite(HVLed, LOW);
}
void HighVoltageToggle(unsigned long i) { //toggle
  if (debounce(millis(), lastHVToggle, 101)) { //has to be precharged
  if (!HVOn) {
  //Serial.println("High Voltage system activating...");
  HVOnInProgress=true;
  digitalWrite(softStartOut, HIGH); //make sure this works
  //Serial.println("- Soft start enabled.");
  } else {
  //off sequence
  //Serial.print("Disabling High Voltage system...");
  digitalWrite(HighVOut, LOW);
  //turning off
  if (HVOnCo == HIGH) {//might be low (aka pull-down) instead
    //digitalWrite(HVLed, LOW);
    //Serial.println("High Voltage shutoff successful.");
  } //else Serial.println("High Voltage shutoff unsuccessful; confirmation discrepancy."); //ends shutoff sequence
  HVOn = false;
  }
  }
  lastHVToggle = millis(); //debounce logging to ensure hv doesnt die from button bouncing. also logs time it was activated for logic things
}
void HighVoltageMain() {
  if (!HVOn && HVOnInProgress && millis()-lastHVToggle>buffer) {
  //turning on
  //waits some set number of seconds before turning on hv to ensure soft start has enough time to enable
    digitalWrite(HighVOut, HIGH); //turning ON HV
    //Serial.println("- High Voltage enabled. Confirming..."); //make this run ONCE while waiting the one second
    //gives system some time to activate fully before checking
    if(millis()-lastHVToggle>buffer) {
      if (digitalRead(HVOnCo) == LOW) { //( PULL UP)
        HVOn = true;
        HVOnInProgress=false;
        //turns off soft start because high voltage is now on fully
        digitalWrite(softStartOut, LOW); //turning off soft start
        //feedback for driver to actually know hv is on
        //digitalWrite(HVLed, HIGH);
        //Serial.println("High Voltage activation confirmed. Soft start deactivated.");
        //ends activation sequence
      } //else Serial.println("High Voltage activation not confirmed.");
    }
  }
}
void SolarPanelOff() {
  PVcanEnable = !digitalRead(PVOffButton);
  if (!PVcanEnable) {
  PV2On = false;
  PV1On = false;
  digitalWrite(PV2Out, LOW);
  digitalWrite(PV1Out, LOW);
  }
}
void SolarPanel1(unsigned long i) { //toggle (solar panels dont need debounce as they only go on or all off!!!)
  if (PVcanEnable && HVOn) {
  if (debounce(millis(), lastPV1On, 100)) {
  PV1On = !PV1On;
  digitalWrite(PV1Out, PV1On);
  lastPV2On = millis();
  }
  }
}
void SolarPanel2(unsigned long i) { //toggle (solar panels dont need debounce as they only go on or all off!!!)
  if (PVcanEnable && HVOn) {
      if (debounce(millis(), lastPV2On, 100)) {
      PV2On = !PV2On;
      digitalWrite(PV2Out, PV2On);
      lastPV2On = millis();
      }
  }
}
