
// include the library code:
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include "Motor.h"


// serialmodes
#define SETUP 0
#define GS232A 1
#define EASYCOM 3
int serialMode=SETUP;

#define PROMPT 0
#define SHOWAEC 1
unsigned char serialSubMode=PROMPT;

#define SERIALBUFFERSIZE 64
char serialBuffer[SERIALBUFFERSIZE];
unsigned int serialBufferWriteIndex=0;
unsigned int serialBufferReadIndex=0;
unsigned int serialBufferCount=0;

/**
 * azi / ele raw value sampling buffers
 */
#define SAMPSIZE 16
unsigned int aziRaw[SAMPSIZE];
unsigned int eleRaw[SAMPSIZE];
unsigned int rawIndex=0;


float aziSp0[2]={0.0,0.0}; // raw-->degree, low
float aziSp1[2]={395.0,270.0}; // raw-->degree, hi

float eleSp0[2]={0.0,0.0}; // raw-->degree, low
float eleSp1[2]={574.0,90.0}; // raw-->degree, hi


#define LEFT 9
#define RIGHT 10
#define DOWN 11
#define UP 12

// initialize the library with the numbers of the interface pins

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);


struct eepromValues{
  int az0[2],az1[2];
  int el0[2],el1[2];
  int mode;           // last mode setted
  int checksum;
};


void storeEE(){
  struct eepromValues val;
  int i;

    
  val.mode = serialMode;
  val.az0[0]=(int)aziSp0[0];
  val.az0[1]=(int)aziSp0[1];
  val.az1[0]=(int)aziSp1[0];
  val.az1[1]=(int)aziSp1[1];

  val.el0[0]=(int)eleSp0[0];
  val.el0[1]=(int)eleSp0[1];
  val.el1[0]=(int)eleSp1[0];
  val.el1[1]=(int)eleSp1[1];

  int chk=0;
  char *p = (char *)&val;
  for (int i=0;i<sizeof(struct eepromValues)-sizeof(int); i++){
    chk=chk+p[i]+i;    
  }
  val.checksum = chk;
  
  byte *pt = (byte*)&val;
  for (i=0;i<sizeof(val); i++){
    EEPROM.write (i,pt[i]);
  }
}

int retrieveEE(){
  struct eepromValues val;
  int i;
  int checksum;
    
  byte *pt = (byte*)&val;
  for (i=0;i<sizeof(val); i++){
    pt[i] = EEPROM.read (i);
  }

  int chk=0;
  char *p = (char *)&val;
  for (int i=0;i<sizeof(struct eepromValues)-sizeof(int); i++){
    chk=chk+p[i]+i;    
  }

    
  if (val.checksum == chk){
    serialMode = val.mode;
    aziSp0[0] = (float) val.az0[0];
    aziSp0[1] = (float) val.az0[1];

    aziSp1[0] = (float) val.az1[0];
    aziSp1[1] = (float) val.az1[1];

    eleSp0[0] = (float) val.el0[0];
    eleSp0[1] = (float) val.el0[1];

    eleSp1[0] = (float) val.el1[0];
    eleSp1[1] = (float) val.el1[1];
    return 0;

  }
  return -1;

}


void putInBuffer(char ch){
  serialBuffer[serialBufferWriteIndex] = ch;
  serialBufferWriteIndex++; serialBufferWriteIndex%=SERIALBUFFERSIZE;
  if (serialBufferCount<SERIALBUFFERSIZE) serialBufferCount++;
}

/**
 * return a line terminated with given char, or null if no present
 * todo: return all line if full
 */

char *getBufferLine(char t){

  static char outbuf[SERIALBUFFERSIZE+1];
  unsigned int readindex = serialBufferReadIndex;
  unsigned int count = serialBufferCount;
  unsigned int blen=0;
  if (count==0) return 0;
  while ( count>0) {
    char ch = serialBuffer[readindex];readindex++; readindex%=SERIALBUFFERSIZE;
    count--;
    outbuf[blen++]=ch;
    if (ch==t){
      outbuf[blen-1]=0;
      serialBufferReadIndex = readindex;
      serialBufferCount-=blen;
      return outbuf;
    }
  }
  return 0;

}

unsigned int getAziRaw(){
  unsigned int m=0;
  for (int i=0;i<SAMPSIZE;i++){
    m+=aziRaw[i];
  }
  return m/SAMPSIZE;
}

unsigned int getEleRaw(){
  unsigned int m=0;
  for (int i=0;i<SAMPSIZE;i++){
    m+=eleRaw[i];
  }
  return m/SAMPSIZE;
}


/**
 *get azi/ ele, corrected
 */
int getAzi(){
  float aziraw = (float)getAziRaw();
  
  float a = (aziraw-aziSp0[0])/(aziSp1[0]-aziSp0[0]) * (aziSp1[1]-aziSp0[1])+aziSp0[1];
  return (unsigned int) (a+0.5);

}

int getEle(){
  float eleraw = (float)getEleRaw();  
  float a = (eleraw-eleSp0[0]) / (eleSp1[0]-eleSp0[0]) * (eleSp1[1]-eleSp0[1])+eleSp0[1];
  return (unsigned int) (a+0.5);
}


/** 
 * azimuth state machine
 */
void aziDebug(char *msg){
  Serial.print (F("az>> "));
  Serial.println (msg);
}
void aziFw(){
  digitalWrite(RIGHT, 1);
}
void aziRev(){
  digitalWrite(LEFT, 1);
}
void aziStop(){
  digitalWrite(LEFT, 0);
  digitalWrite(RIGHT, 0);
}
int aziPosition(){
  return getAzi();
}
Motor aziMotor(aziFw,aziRev,aziStop,aziPosition,0);

/** 
 * elevation state machine
 */
void eleDebug(char *msg){
  Serial.print (F("el>> "));
  Serial.println (msg);
}
void eleFw(){
  digitalWrite(UP, 1);
}
void eleRev(){
  digitalWrite(DOWN, 1);
}
void eleStop(){
  digitalWrite(UP, 0);
  digitalWrite(DOWN, 0);
}
int elePosition(){
  return getEle();
}
Motor eleMotor(eleFw,eleRev,eleStop,elePosition,0);


void prompt(){
  Serial.print (F("> "));
}

unsigned long millis();
unsigned long sensorRTime= 0;
unsigned long lcdExpTime=0;
unsigned long smExpTime=0;
unsigned long millisnow;

char tempStr[80];
int dval,dval_raw;

void setup() {
  int ee=retrieveEE();

  Serial.begin(9600);

  pinMode(LEFT, OUTPUT);
  pinMode(RIGHT, OUTPUT);
  pinMode(UP, OUTPUT);
  pinMode(DOWN, OUTPUT);
  digitalWrite(LEFT, 0);
  digitalWrite(RIGHT, 0);
  digitalWrite(UP, 0);
  digitalWrite(DOWN, 0);

  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print(F("SatHole - IU0HUN"));

  // todo: store in eeprom
  eleMotor.setCheckInterval(2000);
  eleMotor.setMinMoveStep(1);
  aziMotor.setApproachStopValue(3);


  if (ee!=0){
    Serial.println (F("Calibration data missing: please calibrate"));
  }
  if (serialMode==SETUP){
    Serial.println (F("Sathole version 0.1"));
    if (!ee) Serial.println (F("Calibration values retrieved from eeprom"));
    prompt();
  }

}

void loop() {

  long sensorValue;
  char * line;
  int dval,dval1,dval_raw;

  millisnow=millis();  
  /*
   *  read analog sensors in circular buffers
   */
  if (millisnow > sensorRTime){
    sensorRTime=millisnow+50;    // read each xx msec
    aziRaw[rawIndex] = analogRead(0);
    eleRaw[rawIndex++] = analogRead(1);
    rawIndex%=SAMPSIZE;    
  }

  // run motor state machines every 100 msec
  if (millisnow > smExpTime){
    smExpTime=millisnow+100;
    aziMotor.sm();
    eleMotor.sm();
  }

  // check unsolecided modes
  if (serialMode==SETUP && serialSubMode==SHOWAEC){
    sprintf (tempStr,"az %d (raw %d) el %d (raw %d)",getAzi(),getAziRaw(),getEle(),getEleRaw());
    Serial.write (0x0d);
    Serial.print (tempStr);
  }

  if (Serial.available()){
    char ch = Serial.read();
    if (ch!=-1) putInBuffer(ch);
    switch (serialMode){
      case EASYCOM:
        break;
      case GS232A:
        line = getBufferLine(0x0d);
        if (line==0) break;
        if (strlen(line)==0) break;
        
        if (strcmp(line,"R")==0){ // turn rotator right
          digitalWrite(RIGHT, 1);        
          digitalWrite(LEFT, 0);        
          Serial.print(F("\r\n"));
        } else 
        if (strcmp(line,"L")==0){ // turn rotator left
          digitalWrite(LEFT, 1);        
          digitalWrite(RIGHT, 0);        
          Serial.print(F("\r\n"));

        } else 
        if (strcmp(line,"U")==0){ // turn rotator up
          digitalWrite(UP, 1);        
          digitalWrite(DOWN, 0);        
          Serial.print(F("\r\n"));

        } else 
        if (strcmp(line,"D")==0){ // turn rotator down
          digitalWrite(UP, 0);        
          digitalWrite(DOWN, 1);        
          Serial.print(F("\r\n"));

        } else 
        if (strcmp(line,"A")==0){ // stop azi
          digitalWrite(LEFT, 0);        
          digitalWrite(RIGHT, 0);        
          Serial.print(F("\r\n"));
          aziMotor.stop();

        } else 
        if (strcmp(line,"E")==0){ // stop elevation
          digitalWrite(UP, 0);        
          digitalWrite(DOWN, 0);        
          Serial.print(F("\r\n"));
          eleMotor.stop();

        } else 
        if (strcmp(line,"S")==0){ // stop command
          digitalWrite(UP, 0);        
          digitalWrite(DOWN, 0);        
          digitalWrite(LEFT, 0);        
          digitalWrite(RIGHT, 0);        
          aziMotor.stop();
          eleMotor.stop();
          Serial.print(F("\r\n"));
        } else 
        if (strcmp(line,"C")==0){ // Return current azimuth  angle in the form "+0nnn" degrees.
          sprintf (tempStr,"+%04d\r\n",getAzi());
          Serial.print (tempStr);
        } else 
        if (strcmp(line,"B")==0){ // Return current elevation  angle in the form "+0nnn" degrees.
          sprintf (tempStr,"+%04d\r\n",getEle());
          Serial.print (tempStr);
        } else 
        if (strcmp(line,"C2")==0){ // Return azimuth and elevation "+0aaa+0eee"
          sprintf (tempStr,"+%04d+%04d\r\n",getAzi(),getEle());
          Serial.print (tempStr);
        } else
        if (sscanf(line,"M%3d",&dval)==1){ // Turn to aaa degrees azimuth
          aziMotor.go(dval);
          Serial.print(F("\r\n"));
        } else 
        if (sscanf(line,"W%3d %3d",&dval,&dval1)==2){ // Turn azi/ele
          aziMotor.go(dval);
          eleMotor.go(dval1);
          Serial.print(F("\r\n"));
        } else 
        if (sscanf(line,"X%1d",&dval)==1){      // speed commands
          Serial.print(F("\r\n"));
        } else 
        if (strcmp(line,"+++")==0){ // return to setup mode
          Serial.println (F("Entering setup"));
          serialMode=SETUP;
          storeEE();
        } else {
          //unknown command
          Serial.print (F("? >\r\n"));
        }
        break;
      case SETUP:
        if (serialSubMode){
          serialSubMode=0;
        }
        // echo char in setup mode
        Serial.write(ch);
        line = getBufferLine(0x0d);

        if (line==0) break;
        if (strlen(line)==0){Serial.println();prompt();break;}
        Serial.println ();
        if ( strcmp (line,"?")==0 || strcmp (line,"h")==0 ){
            printHelp();prompt();
        } else if (strcmp (line,"ae")==0){
            sprintf (tempStr,"az %d (raw %d) el %d (raw %d)",getAzi(),getAziRaw(),getEle(),getEleRaw());
            Serial.println (tempStr);
            prompt();
        } else if (strcmp (line,"gs232a")==0){
            Serial.println (F("Entering gs232a emulation"));
            serialMode=GS232A;
            storeEE();
        } else if (strcmp (line,"aec")==0){
            serialSubMode=SHOWAEC;
        } else if (strcmp (line,"left")==0){
            digitalWrite(LEFT, 1);
            prompt();
        } else if (strcmp (line,"right")==0){
            digitalWrite(RIGHT, 1);
            prompt();
        } else if (strcmp (line,"up")==0){
            digitalWrite(UP, 1);
            prompt();
        } else if (strcmp (line,"down")==0){
            digitalWrite(DOWN, 1);
            prompt();
        } else if (strcmp (line,"stop")==0){
            digitalWrite(LEFT, 0);
            digitalWrite(RIGHT, 0);
            digitalWrite(UP, 0);
            digitalWrite(DOWN, 0);
            prompt();
        } else if (strcmp (line,"dump")==0){
            dtostrf(aziSp0[0], 4, 0, tempStr+60);tempStr[64]=0;
            dtostrf(aziSp0[1], 3, 0, tempStr+70);tempStr[73]=0;              
            sprintf (tempStr,"A0 %s to %s dg",tempStr+60,tempStr+70);
            Serial.println (tempStr);            
            dtostrf(aziSp1[0], 4, 0, tempStr+60);tempStr[64]=0;
            dtostrf(aziSp1[1], 3, 0, tempStr+70);tempStr[73]=0;              
            sprintf (tempStr,"A1 %s to %s dg",tempStr+60,tempStr+70);
            Serial.println (tempStr);            
            dtostrf(eleSp0[0], 4, 0, tempStr+60);tempStr[64]=0;
            dtostrf(eleSp0[1], 3, 0, tempStr+70);tempStr[73]=0;              
            sprintf (tempStr,"E0 %s to %s dg",tempStr+60,tempStr+70);
            Serial.println (tempStr);            
            dtostrf(eleSp1[0], 4, 0, tempStr+60);tempStr[64]=0;
            dtostrf(eleSp1[1], 3, 0, tempStr+70);tempStr[73]=0;              
            sprintf (tempStr,"E1 %s to %s dg",tempStr+60,tempStr+70);
            Serial.println (tempStr);            
            prompt();
        } else if (sscanf(line,"go %d %d",&dval,&dval1)==2){
            // goto to given azi,ele
            aziMotor.go(dval);
            eleMotor.go(dval1);
            prompt();

        } else if (sscanf(line,"az %d",&dval)==1){
            // goto to given azi
            aziMotor.go(dval);
            prompt();
        } else if (sscanf(line,"el %d",&dval)==1){
            // goto to given el
            eleMotor.go(dval);
            prompt();
        } else if (sscanf (line,"set-a0 %d",&dval)==1){
            dval_raw = getAziRaw();
            aziSp0[0] = float(dval_raw);
            aziSp0[1] = float(dval);
            storeEE();
            sprintf (tempStr,"A0 %d dg for %d",dval,dval_raw);              
            Serial.println (tempStr);
            prompt();
        } else if (sscanf (line,"set-a1 %d",&dval)==1){

            aziSp1[0] = float(getAziRaw());
            aziSp1[1] = float(dval);
            storeEE();
            sprintf (tempStr,"A1 %d dg for %d",dval,getAziRaw());              
            Serial.println (tempStr);
            prompt();
        } else if (sscanf (line,"set-e0 %d",&dval)==1){

            eleSp0[0] = float(getEleRaw());
            eleSp0[1] = float(dval);
            storeEE();
            sprintf (tempStr,"E0 %d dg for %d",dval,getEleRaw());              
            Serial.println (tempStr);
            prompt();
        } else if (sscanf (line,"set-e1 %d",&dval)==1){

            eleSp1[0] = float(getEleRaw());
            eleSp1[1] = float(dval);
            storeEE();
            sprintf (tempStr,"E1 %d dg for %d",dval,getEleRaw());              
            Serial.println (tempStr);
            prompt();
        } else if (strcmp (line,"factory-reset")==0){
            for (int i=0;i<sizeof(struct eepromValues); i++){
              EEPROM.write (i,0xaa);
            }
            delay(100);
            asm volatile ("  jmp 0"); 
        } else {
            Serial.println (F("Invalid command, h or ? for help"));
            prompt();
        }
        
        break;
      default:
        break;
    }
  }


// update lcd display
  if ( millis()>lcdExpTime){
    lcdExpTime = millis()+100;

    lcd.setCursor(0, 1);

    lcd.setCursor(0, 1);
    sprintf (tempStr," Az %3d El %3d",getAzi(),getEle());
    lcd.print(tempStr);
  }
}

void printHelp(){
  Serial.println (F("IU0HUN rotor Interface \"Sat Hole\""));
  Serial.println (F("List of commands in setup mode"));

  Serial.println (F("*** STATUS commands"));
  Serial.println (F("ae return azimuth and elevation"));
  Serial.println (F("aec return azimuth and elevation, continuous mode"));

  Serial.println (F("*** move commands"));
  Serial.println (F("left go left"));
  Serial.println (F("right go right"));
  Serial.println (F("up go up"));
  Serial.println (F("down go down"));
  Serial.println (F("az <angle> go to azimuth <angle>"));
  Serial.println (F("el <angle> go to elevation <angle>"));
  Serial.println (F("go <azimuth> <elevation> go to <azimuth>, <elevation>"));
  Serial.println (F("stop stop motors"));

  Serial.println (F("dump dump current configuration"));
  Serial.println (F("set-a0 <angle> set start azimuth angle, in degrees, for current azimuth"));
  Serial.println (F("set-a1 <angle> set end azimuth angle, in degrees, for current azimuth"));

  Serial.println (F("set-e0 <angle> set start elevation angle, in degrees, for current elevation"));
  Serial.println (F("set-e1 <angle> set end elevation angle, in degrees, for current elevation"));

  Serial.println (F("*** emulation commands"));
  Serial.println (F("easycom enter easycom emulation mode, and set as default"));
  Serial.println (F("gs232a enter gs232a emulation mode, and set as default"));

  Serial.println (F("factory-reset factory-reset device"));
}
