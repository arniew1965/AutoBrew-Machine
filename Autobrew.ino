//**************************************************************
//                          AutoBrew                           *
//                                                             *
//     Particle Photon based Automated Brewing Machine         *
//    by Arnie Wierenga arniew1965 (at) gmail (dot) com        *
//                                                             *
// This machine is connected to the Internet of Things         *
// and uses a web application as its GUI. In order to use      *
// AutoBrew, the machine must be connected via wifi            *
// to the internet.                                            *
//                                                             *
// Claiming a device is covered in documentation by Particle.  *
//                                                             *
// AutoBrew 200                                                *
//                                                             *
// Changelog to 0.200                                          *
// ==================                                          *
// complete redaction of previous code                         *
//                                                             *
//**************************************************************

//================== # include Libraries =======================
#include "Particle-OneWire.h"
#include "particle-dallas-temperature.h"

//============== # define inputs and outputs ===================
#define ONE_WIRE_BUS 2  // Setup oneWire for any OneWire device
OneWire oneWire(ONE_WIRE_BUS); // Pass reference to Dallas Temperature.
DallasTemperature sensor(&oneWire);

//================== # define CONSTANTS ========================

// program steps
int IDLE=0;
int INFUSE=1;
int MASH1=2;
int MASH2=3;
int MASH3=4;
int MASH4=5;
int MASH5=6;
int BOIL=7;

//================== # declare variables =======================

// define hardware
int oServo=D0;  //dropper servo
int oPump=D3;   //liquor flow pump
int oHeat=D7;   //heater (solid state)
Servo dropper;  // Setup dropper servo
/* dropper starting position is 0
   addition drops are in positions 1, 2, 3 (possibly 4)
   all droppers are capped after loading
   ADDN_SERVO_ANGLE_START=94 | eeprom address for drop 0 angle
   94+4=98 | drop 1 angle etc. */

// system strings
String strVer="AB 0.200.170131";
String strStatus;
String strMsg;

//default parameters
boolean blnTempUnitC=true;
int iBoilTrig=975;
int iInfuseDiff=10;
int iMashOutTemp=760;
int iMashOutTime=1;

//process variables
int iStep;
int iSecs;
int iAddn;                  //points to current drop position
int iAddnAngle;             //servo angle for current drop
int iProcessInterval = 5;   //seconds
int iTimeStepEnds=0;         //0 - timer inactive; >0 indicates expiry time
int iDropTime[]={0,0,0,0};
int iDropDeg[]={0,0,0,0};
int iPublishTime=0;         //publish event status
int iSaveTime=0;            //recovery data to EEPROM
boolean blnHeaterReqd=false;
int iHeaterOnUntil=1;       //used to control % heating of boil
int iHeaterOffUntil=0;      //used to control % heating of boil
boolean blnTimeSet=false;   //used to doStepCheck()
boolean blnTempSet=false;   //used to doStepCheck()
int iActiveDropCup=0;
int itimeLastAlive;
boolean blnVerbose=false;   //extra items are published on status string during verbose

// EEPROM

// 0-3 EEPROM header - (integer)

// Recovery data
// =============
// 114      blnTimeSet
// 115      blnTempSet
// 116-119  iTempTarg
// 9-12 Current Step - (integer)
// 13-16 Step Expiry Time - (integer)
// 17-20 Last Alive Time - (integer)

// Recipe steps
// ============
// 21-24 Infuse Temp - (integer)
// 25-28 Infuse Time - (integer)
// 29-32 Mash1 Temp - (integer)
// 33-36 Mash1 Time - (integer)
// 37-40 Mash2 Temp - (integer)
// 41-44 Mash2 Time - (integer)
// 45-48 Mash3 Temp - (integer)
// 49-52 Mash3 Time - (integer)
// 53-56 Mash4 Temp - (integer)
// 57-60 Mash4 Time - (integer)
// 61-64 Mash5 Temp - (integer)
// 65-68 Mash5 Time - (integer)
// 69-72 Boil Time - (integer)
// 73-76 Add1 Time - (integer)
// 77-80 Add2 Time - (integer)
// 81-84 Add3 Time - (integer)
// 85-88 Add4 Time - (integer)

// Machine settings
// ================
// 89 Temp unit C/F - (boolean)
// 90-93 Boil trigger - (integer)
// 94-97 DropAngle0 - (integer)
// 98-101 DropAngle1 - (integer)
// 102-105 DropAngle2 - (integer)
// 106-109 DropAngle3 - (integer)
// 110-113 DropAngle4 - (integer)

int EEPROM_CURR_STEP=9;     //4 bytes (iStep)
int EEPROM_STEP_EXPIRES=13; //4 bytes (iStepExpires)
int EEPROM_LAST_ALIVE=17;   //4 bytes (iLastAlive)
int EEPROM_TIME_SET=114;    //1 byte (blnTimeSet)
int EEPROM_TEMP_SET=115;    //1 byte (blnTempSet)
int EEPROM_TEMP_TARG=116;    //4 bytes (iTempTarg)

int EEPROM_HEADER=0;        //4 bytes (iHeader)
int EEPROM_TEMP_UNIT=89;     //1 byte (blnTempUnitC)
int EEPROM_BOIL_TRIG=90;     //4 bytes (iBoilTrig)
int EEPROM_DROP_ANGLE0=94;   //4 bytes (iBoilTrig)

// Temperature
float  fTemp= 1.0;
int iTempTarg = 0; // values in degrees x 10
int iTempAct = 0;   // values in deg x 10 (999 = 99.9 degrees F or C)
int iReadFail=0;  //counts number of temperature read fails.
int iTempReadTime=0;  //time when next temp read due

void setup()
{
  // set up inputs and outputs
  pinMode(oPump, OUTPUT);
  pinMode(oHeat, OUTPUT);
  dropper.attach(oServo);

  // Dallas temperature sensor
  sensor.begin();
  sensor.setResolution(11);    //9=0.5; 10=0.25; 11=0.125; 12=0.0625 deg C

  doStartUpCheck(); //format, recovery, default values, recipe

  Particle.function("control", remCommand);
  Particle.variable("ABmessage", strMsg);

  // Set local Time Zone
  Time.zone(+11);
}

void loop()
{

  // ***********************************
  // **         check STATUS          **
  // ***********************************

  doProcessCheck();  //updates step, pump, heater, dropper, publishes stats & stores recovery data
  doTempRead();      //updates temperature every 5 seconds
}

void doProcessCheck()
{
  doStepCheck();
  doPumpCheck();
  doHeaterCheck();
  doDropperCheck();
  doStatusPublish();    //publishes to event update every 15 seconds
  doRecoverySave();     //saves recovery data to EEPROM every minute
}

void doStepCheck()
{
  if (iStep==INFUSE)
  {
    if (!blnTimeSet)
    {
      setStepTime(-1);
      iPublishTime=Time.now();
      blnTempSet=false;
    }
    if (blnTimeSet && iTimeStepEnds <= Time.now())
    {
      if (!blnTempSet)
      {
        setStepTarg(-1);
        iPublishTime=Time.now();
      }
    }
    if (blnTempSet && iTempTarg<=iTempAct)
    {
      iStep=MASH1;
      blnTimeSet=false;
      blnTempSet=false;
      iTempTarg=0;
      iTimeStepEnds=0;
      iPublishTime=Time.now();
    }
  }
  else if (iStep>=MASH1 && iStep<=BOIL)
  {
    if (!blnTempSet)
    {
      setStepTarg(-1);
      iPublishTime=Time.now();
    }
    if (blnTempSet && iTempTarg<=iTempAct)
    {
      if (!blnTimeSet)
      {
        setStepTime(-1);
        iPublishTime=Time.now();
      }
    }
    if (blnTimeSet && iTimeStepEnds <= Time.now())
    {
      if (iStep==BOIL)
      {
        iStep=IDLE;
        blnTimeSet=false;
        blnTempSet=false;
        iTempTarg=0;
        iTimeStepEnds=0;
        resetAll();
        iPublishTime=Time.now();
      }
      else
      {
        iStep++;
        blnTimeSet=false;
        blnTempSet=false;
        iTempTarg=0;
        iTimeStepEnds=0;
        iPublishTime=Time.now();
      }
    }
  }

}

void setStepTime(int timeVal)
{
  if (timeVal==-1)
  {
    //load value for step from EEPROM
    int timeAddr;
    if(iStep>=INFUSE && iStep<=MASH5)
    {
      timeAddr=(iStep-1)*8 + 25;
    }
    else if(iStep==BOIL)
    {
      timeAddr=69;
      getDropTimes();
    }
    EEPROM.get(timeAddr, timeVal);
  }
  else
  {
    //value is set via remCommand()
    //if we are setting a delay for infusion step, clear temperature target
    if (iStep==INFUSE)
    {
      blnTempSet=false;
      iTempTarg=0;
    }
  }
  iTimeStepEnds=Time.now()+(timeVal*60);
  blnTimeSet=true;
}

void setStepTarg(int targVal)
{
  if (targVal==-1)
  {
    //load value for step from EEPROM
    if(iStep>=INFUSE && iStep<=MASH5)
    {
      int tempAddr=(iStep-1)*8 + 21;
      EEPROM.get(tempAddr, iTempTarg);
    }
    else if(iStep==BOIL)
    {
      iTempTarg=iBoilTrig;
    }
  }
  else
  {
    iTempTarg=targVal;
  }
  blnTempSet=true;
}

void getDropTimes()
{
  int timeAddr=73;
  int stepMins;
  for (int drop=1; drop<=3; drop++)
  {
    EEPROM.get(timeAddr, stepMins);
    iDropTime[drop]=(stepMins*60) + 10; //10 seconds added to ensure all drops happen
    timeAddr+=4;
  }
}

int getSecs()
{
  if (iTimeStepEnds < Time.now()) //step has expired
  {
    return 0;
  }
  else
  {
    return iTimeStepEnds-Time.now();
  }

}
void doTempRead()
{
  if (iTempReadTime<=Time.now()) readTemp();
}

void doPumpCheck() //updates pump Output
{
if (iStep>=MASH1 && iStep<=MASH5)
  {
    digitalWrite(oPump, HIGH);
  }
  else
  {
    digitalWrite(oPump, LOW);
  }
}

void doHeaterCheck()  //updates heater Output
{
  //Use duty cycle to adjust power during BOIL
  if (iStep==BOIL)
  {
    if (iHeaterOffUntil<iHeaterOnUntil && iHeaterOnUntil<=Time.now())
    {
        //engage off part of duty cycle
        iHeaterOffUntil=Time.now() + 2;  //add two seconds to current time
        blnHeaterReqd=false;
    }
    if (iHeaterOnUntil<iHeaterOffUntil && iHeaterOffUntil<=Time.now())
    {
      //engage on part of duty cycle
      iHeaterOnUntil=Time.now() + 6;   //add 6 seconds to current time
      blnHeaterReqd=true;
    }
  }
  if (iTempAct < iTempTarg) //if temp actual < temp target, apply Heat
  {
    blnHeaterReqd=true;
    //digitalWrite(oHeat, HIGH);
  }
  else  //temperature is reached, heater is off (except during boil)
  {
    if (!iStep==BOIL) {blnHeaterReqd=false;}
    //digitalWrite(oHeat, LOW);
  }

  if (blnHeaterReqd) {digitalWrite(oHeat, HIGH);}
  else {digitalWrite(oHeat, LOW);}
}

void getDropperDegs()
{
  for (int drop=0; drop<=3; drop++)
  {
    EEPROM.get(EEPROM_DROP_ANGLE0 + (drop)*4, iDropDeg[drop]);
  }
}

void doDropperCheck()
{
  int currDrop=0;
  if (iStep==BOIL && blnTimeSet)
  {
    for (int drop=1; drop<=3; drop++)
    {
      if (iDropTime[drop]>=timeLeft()) currDrop++;
    }
    if (iActiveDropCup!=currDrop) setDropperByNum(currDrop);
  }
}

void setDropperByNum(int dropNum)
{
  // dropper starts in position 0
  // first drop is 1
  // last available drop is 3

  if (dropNum>3){return;}  //invalid drop number
  int dropPosn=iDropDeg[dropNum];
  dropper.write(dropPosn);
  iActiveDropCup=dropNum;
}

void doStatusPublish() //publishes event update every 15 secs
{
  if (iPublishTime<Time.now())
  {
    strStatus="$T" + String(iTempAct) + "[";
    strStatus+= String(iTempTarg) + "] ";
    strStatus+= String(timeLeft()) + "s, ";
    if (iStep==INFUSE) strStatus+= "Infuse";
    else if (iStep>=MASH1 && iStep<=MASH5) strStatus+= "Mash" + String(iStep-1);
    else if (iStep==BOIL)
    {
      strStatus+= "Boil - [";
      strStatus+= String(iActiveDropCup) + "]";
    }
    else strStatus+= "Idle";

    if (blnVerbose) {
      strStatus+= ", LA " + String(Time.now()-itimeLastAlive) + "s, ";
      strStatus+=" stepTrig[" ;
      if (blnTimeSet) { strStatus+= "C/"; }
      else { strStatus+= "c/"; }
      if (blnTempSet) { strStatus+= "T] "; }
      else { strStatus+= "t] "; }
      strStatus+=" X[" + String(iReadFail) + "]";
      strStatus+=" dropDeg[" ;
      for (int drop=0; drop<=3; drop++)
      {
        strStatus+= String(iDropDeg[drop]);
        if (drop<3) strStatus+= ",";
      }
      strStatus+="]" ;
    }
    strStatus+= "#";

    Particle.publish("status", strStatus, PRIVATE);
    iPublishTime=Time.now()+15;
  }
}

void doRecoverySave() //saves recovery data to EEPROM every minute
{
  if (iSaveTime<Time.now())
  {
    EEPROM.put(EEPROM_CURR_STEP,iStep);
    EEPROM.put(EEPROM_STEP_EXPIRES,iTimeStepEnds);
    EEPROM.put(EEPROM_LAST_ALIVE,Time.now());
    EEPROM.put(EEPROM_TIME_SET,blnTimeSet);
    EEPROM.put(EEPROM_TEMP_SET,blnTempSet);
    EEPROM.put(EEPROM_TEMP_TARG,iTempTarg);
    iSaveTime=Time.now()+60;
    itimeLastAlive=Time.now();
  }

}

void readTemp()
{
  iTempReadTime=Time.now() + 5; //set next read time

  //read temperature off probe
  sensor.requestTemperatures(); // Send the command to get temperature
  if (blnTempUnitC)
  {
    fTemp=sensor.getTempCByIndex(0);
  }
  else
  {
    fTemp=sensor.getTempFByIndex(0);
  }
  int testTempVal;
  testTempVal= fTemp*10;
  if (testTempVal!=-1270){
    //reading is valid, update register
    iTempAct = fTemp * 10;
  }
  else
  {
    //register a read fail
    iReadFail++;
  }
}

int timeLeft()
{
  int timeLeft=iTimeStepEnds-Time.now();
  if (timeLeft>0) return timeLeft;
  else return 0;
}

void doStartUpCheck()
{
  //check for right EEPROM header
  int header=0;
  EEPROM.get(EEPROM_HEADER,header);
  if (header!=200) resetEEPROM();

  //get recovery data
  int lastAlive=0;
  EEPROM.get(EEPROM_CURR_STEP,iStep);
  EEPROM.get(EEPROM_STEP_EXPIRES,iTimeStepEnds);
  EEPROM.get(EEPROM_LAST_ALIVE,lastAlive);
  EEPROM.get(EEPROM_TIME_SET,blnTimeSet);
  EEPROM.get(EEPROM_TEMP_SET,blnTempSet);
  EEPROM.get(EEPROM_TEMP_TARG,iTempTarg);

  //get default values
  //dropper angles and drop times
  for (int drop=0; drop<=3; drop++)
  {
    EEPROM.get(EEPROM_DROP_ANGLE0+(drop)*4,iDropDeg[drop]);
  }
  getDropTimes();

  //evaluate recovery data
  if (iStep<0 || lastAlive<0 || iTimeStepEnds<0)
  {
    //the data is not valid, perhaps from an EEPROM reset
    resetAll();
    strMsg="Reset because EEPROM data is not valid";
  }
  if (iStep>IDLE && iStep<=BOIL)  //this is an active brew
  {
    int offTime = Time.now()-lastAlive;
    if (offTime<0 || offTime>600)
    {
      //the machine has been off for more than 10 minutes.
      //suspend brewing
      resetAll();
      strMsg="Reset due to extended off time: " + String(offTime) + "s";
    }
    else
    {
      //add some recovery time to steps
      iTimeStepEnds+= Time.now()-lastAlive;
      strMsg=String(offTime) + "s of recovery time is being added to steps";
    }
  }
  else
  {
    iStep=0;
  }
}

void resetEEPROM()
{
  boolean emptyData=true;
  //resets EEPROM and sets current version header
  for (int addr=0; addr<=150; addr++)
  {
    EEPROM.put(addr,emptyData);
  }
  EEPROM.put(EEPROM_HEADER,200);
}

void resetAll()
{
  setDropperByNum(0);
  iStep=IDLE;
  iTempTarg=0;
  iTimeStepEnds=0;
  iSaveTime=0;
  blnHeaterReqd=false;
  EEPROM.put(EEPROM_CURR_STEP,iStep);
  EEPROM.put(EEPROM_STEP_EXPIRES,iTimeStepEnds);
  EEPROM.put(EEPROM_LAST_ALIVE,iSaveTime);
}

void getRecipeFromEEPROM()
{
  strMsg="R:";
  int tempItem=0;
  int timeItem=0;
  for (int addr=21; addr<=61;addr=addr+8)
  {
    EEPROM.get(addr, tempItem);
    EEPROM.get(addr+4, timeItem);
    strMsg+= String(tempItem) +"," + String(timeItem) + ";";
  }
  EEPROM.get(69, timeItem);
  strMsg+= String(timeItem) + ";";
  for (int addr=73; addr<=81; addr=addr+4)
  {
    EEPROM.get(addr, timeItem);
    strMsg+= String(timeItem) + ",";
  }
}

int remCommand(String strCmdLine) //human readable user interface
{
  // 'r': toggle start and reset
  // 'R': get current recipe
  // 'Rt,T;t,T;...': set complete recipe
  // 'cxxxx': set clock mins
  // 'txxxx': set Target temperature
  // 'd[num]': set servo to drop position [num] (0-4)
  // 'd[num],xxx': set servo angle of drops 1-4
  // 'v': send AB version
  // 'u': toggle Temp unit Farenheit/Celcius
  // 'bxxxx': sets boil trigger
  // 'b': returns boil trigger
  // 'V': toggles verbose mode

  // with a remote command, immediately update status
  iPublishTime=Time.now();

  if (strCmdLine.startsWith("r"))
  {
    if (iStep==0)
    {
      iStep=INFUSE;
      blnTimeSet=false;
      blnTempSet=false;
      strMsg="Starting Program";
    }
    else
    {
      iStep=IDLE;
      blnTimeSet=false;
      blnTempSet=false;
      resetAll();
      strMsg="Resetting Program";
    }
    //loadStepValues(iStep);
    return iStep;
  }
  else if (strCmdLine.startsWith("R"))
  {
    if (strCmdLine.length()==1) //request recipe
    {
      getRecipeFromEEPROM();
      return 2;
    }
    else  //set recipe
    {
      int tempItem;
      int timeItem;
      int end;
      strCmdLine=strCmdLine.substring(1); //remove first character
      for (int addr=21; addr<=21; addr=addr+8)
      {
        end = strCmdLine.indexOf(",");
        tempItem=strCmdLine.substring(0, end).toInt();
        strCmdLine=strCmdLine.substring(end + 1); //remove used string
        end = strCmdLine.indexOf(";");
        timeItem=strCmdLine.substring(0,end).toInt();
        EEPROM.put(addr, tempItem);
        EEPROM.put(addr+4, timeItem);
        strCmdLine=strCmdLine.substring(end + 1); //remove used string
      }
      for (int addr=29; addr<=61; addr=addr+8)
      {
        end = strCmdLine.indexOf(",");
        tempItem=strCmdLine.substring(0, end).toInt();
        strCmdLine=strCmdLine.substring(end + 1); //remove used string
        end = strCmdLine.indexOf(";");
        timeItem=strCmdLine.substring(0,end).toInt();
        EEPROM.put(addr, tempItem);
        EEPROM.put(addr+4, timeItem);
        strCmdLine=strCmdLine.substring(end + 1); //remove used string
      }
      for (int addr=69; addr<=69; addr=addr+4)
      {
        end = strCmdLine.indexOf(";");
        timeItem=strCmdLine.substring(0,end).toInt();
        EEPROM.put(addr, timeItem);
        strCmdLine=strCmdLine.substring(end + 1); //remove used string
      }
      for (int addr=73; addr<=81; addr=addr+4)
      {
        end = strCmdLine.indexOf(",");
        timeItem=strCmdLine.substring(0,end).toInt();
        EEPROM.put(addr, timeItem);
        strCmdLine=strCmdLine.substring(end + 1); //remove used string
      }
      getRecipeFromEEPROM();
      return 2;
    }
  }
  else if (strCmdLine.startsWith("c"))
  {
    int setMins;
    String strTarg=strCmdLine.substring(1);
    setMins=strTarg.toInt();
    setStepTime(setMins);
    return setMins;
  }
  else if (strCmdLine.startsWith("t"))
  {
    String strTarg=strCmdLine.substring(1);
    setStepTarg(strTarg.toInt());
    return iTempTarg;
  }
  else if (strCmdLine.startsWith("d"))
  {
    int numDrops;
    if (strCmdLine.length()==2)
    {
      //set drop position
      strCmdLine=strCmdLine.substring(1); //remove first character
      int dropNum=strCmdLine.toInt();
      setDropperByNum(dropNum);
      strMsg="Dropper set to [" + String(dropNum) + "]";
      return iDropDeg[dropNum];
    }
    else if (strCmdLine.length()>3)
    {
      //set the drop angles
      //strCmdLine=strCmdLine.substring(1); //remove first character
      //strMsg="Drop[" + strCmdLine.substring(0,2) + "]";
      int dropNum=strCmdLine.substring(1,2).toInt();
      int dropAngle=strCmdLine.substring(3).toInt();
      EEPROM.put(EEPROM_DROP_ANGLE0+(dropNum)*4,dropAngle);
      iDropDeg[dropNum]=dropAngle;
      setDropperByNum(dropNum);
      strMsg="Drop[" + String(dropNum) + "] set to " + String(dropAngle) + " degrees";
      return iDropDeg[dropNum];
    }
  }
  else if (strCmdLine.startsWith("v"))
  {
    strMsg="AB ver: " + strVer;
    return 2;
  }
  else if (strCmdLine.startsWith("u"))
  {
    strMsg="Temperature unit: ";
    if (blnTempUnitC==false)
    {
      blnTempUnitC=true;
      strMsg=strMsg + "Celcius";
    }
    else
    {
      blnTempUnitC=false;
      strMsg=strMsg + "Farenheit";
    }
    EEPROM.put(EEPROM_TEMP_UNIT,blnTempUnitC);
    return 2;
  }
  else if (strCmdLine.startsWith("b"))
  {
    if (strCmdLine.length()==1)
    {
      //return boil trigger value
      EEPROM.get(EEPROM_BOIL_TRIG,iBoilTrig);
      strMsg="Boil trigger: " + String(iBoilTrig);
      return iBoilTrig;
    }
    else
    {
      //save boil trigger value
      String strTarg=strCmdLine.substring(1);
      iBoilTrig=strTarg.toInt();
      EEPROM.put(EEPROM_BOIL_TRIG,iBoilTrig);
      strMsg="Boil trigger set: " + String(iBoilTrig);
      return iBoilTrig;
    }
  }
  else if (strCmdLine.startsWith("V"))
  {
    //toggle verbose status updates
    blnVerbose=!blnVerbose;
    if (blnVerbose) return 1;
    if (!blnVerbose) return 0;
  }
  else
  {
    return 0;
  }
  Particle.publish("remCommand", strMsg, PRIVATE);
}
