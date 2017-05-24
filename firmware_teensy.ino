  //------------------------------------------------------------------------------------------------------------
// Revolution Lighting Project Firmware
//
// Todo
// (1) resolve timeout/crash issue after mode set
// (2) implement/verify/debug Soft reboot (RPi Pin12/GPIO18 toggle) - Done
// (3) implement/verify/debug Hard reboot (RPi RUN toggle) - Done
// (4) implement code to ignore duplicate node gets - Done
// (5) resolve event handler crashes - Done
// (6) implement/debug fail safe (soft/feather and hard/hammer reboot logic) - Done
// (7) comment/refactor code for readability - Started
//------------------------------------------------------------------------------------------------------------
#define RELEASENUMBERMAJOR 0
#define RELEASENUMBERMINOR 1
#define LV_HV 1 //0 for LV, 1 for HV
#include <i2c_t3.h>


//------------------------------------------------------------------------------------------------------------
// Function prototypes for event handlers
//------------------------------------------------------------------------------------------------------------
void receiveEvent(size_t count);  // I2C Receive event handler
void requestEvent(void);          // I2C Request data event handler

//------------------------------------------------------------------------------------------------------------
// Memory
//------------------------------------------------------------------------------------------------------------
#define MEM_LEN 256
#define VERSION 0
#define PWMSET 1
#define PLC 2
#define OPOL 3
#define AIN_DRIVE 4
#define AIN 5
#define DC 6
#define IMEAS 7
#define HVDIMMODE 8
#define VIN 9
#define SR_CLK 19
#define SR_DAT 17
#define SR_LAT 18

//------------------------------------------------------------------------------------------------------------
// Globals for Teensy-to-Pi Rest Interface
//------------------------------------------------------------------------------------------------------------
//#define PI_GPIO18_PIN12 32                            // Teensy Pin32 (TP2) jumper wired to Rpi GPIO18-pin12 (TP21)
//#define PI_RUN_RESET 31                               // Teensy Pin23 to RPi Run (GPIO31)
boolean ReceivedAnyI2cFromPi = false;                   // Set True first time Teensy receives data from Pi
unsigned long bootUpMillis;                             // millis when setup() ran
unsigned long rebootWaitTimeMillis = 45000;             // 45 seconds
unsigned long timeStampMostRecentRx;                    // loop(): Zero indicates no Rx event received yet
unsigned long timeSinceMostRecentRx;                    // loop(): Zero indicates no Rx event received yet
unsigned long timeDeltaRx = 0;                          // loop(): 

// Used in received event handler only
unsigned long nTimeStampMostRecentRx;                   // Used in rx event handler
unsigned long nTimeDeltaRx = 0;                         // Used inside of event handler
unsigned long nTimeStampPriorRx;                        //
unsigned long currentMillis = 0;                        //
int softRebootTryNum = 0;                               // 
int maxNumSoftRebootTries = 3;                          // 

//------------------------------------------------------------------------------------------------------------
// Globals Pin Definitions
//------------------------------------------------------------------------------------------------------------

const int PI_RUN_RESET = 31;                            // Hard reboot pin 
const int PI_GPIO18_PIN12 = 32;                         // Soft reset pin

const int pwmPin[] = {2,3,4,5,6,9,10,29};               //
const int analogIn[] = {A22,A0,A1,A2};                  //
const int pSource[] = {27,28,11,13};                    //
const int ainDrive[] = {21,22,23,30};                   //
const int dc[] = {12,24,25,26};                         //
const int iMeas[] = {A14,A15,A16,A17,A18,A19,A20,A21};  //

uint8_t ignoreLastRx=0;                                 // ***
uint8_t databuf[MEM_LEN];                               // databuf[256] ***
volatile uint8_t received;                              // ***
uint8_t addr;                                           // ***
int dimmer0_10Vmode = 0x00;                             // ***
int dimmerEdge = 0x00;                                  // ***
uint8_t dimmerRelayState = 0x00;                        // ***
uint16_t pwmData[] = {0x0000,0x0000,0x0000,0x0000,      // ***
                      0x0000,0x0000,0x0000,0x0000};

//------------------------------------------------------------------------------------------------------------
// Setup
//------------------------------------------------------------------------------------------------------------
void setup()
{
    int i;
    pinMode(1,OUTPUT);
    pinMode(SR_CLK,OUTPUT);
    pinMode(SR_LAT,OUTPUT);
    pinMode(SR_DAT,OUTPUT);
    pinMode(PI_RUN_RESET,OUTPUT);            
    pinMode(PI_GPIO18_PIN12, OUTPUT);       

    for (i=0;i<sizeof(pSource);i++)
    {
      pinMode(pSource[i],OUTPUT);
      digitalWrite(pSource[i],LOW);
    }
    
    for (i=0;i<sizeof(pwmPin);i++)
    {
      pinMode(pwmPin[i],OUTPUT);
      analogWriteFrequency(pwmPin[i],200);
      analogWrite(pwmPin[i] , 0);
    }

    for (i=0;i<sizeof(dc);i++)
    {
      pinMode(dc[i],INPUT);
    }
    for (i=0;i<sizeof(iMeas);i++)
    {
      pinMode(iMeas[i],INPUT);
    }
    for (i=0;i<sizeof(ainDrive);i++)
    {
      pinMode(ainDrive[i],OUTPUT);
    } 
    //pinMode(vinSense,INPUT);
    analogWriteResolution(16);
    analogReadResolution(12);
    analogReadAveraging(256);
   
    // Data init
    received = 0;
    memset(databuf, 0, sizeof(databuf));
 
    Serial.begin(115200);
    Serial.println("Teensy Setup has run! We've Booted!");
        
    //delay must take place before interupt handlers are enabled
    //interupt handler ignore delay statementswait 45 seconds
    bootUpMillis = millis();
    Serial.println("Waiting 45 seconds");
    delay(rebootWaitTimeMillis);
    Serial.println("45 seconds since Teensy booted");
    delay(1000);

// Register wire/I2c slave received and request handlers
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

 // Setup for Slave mode, address 0x66, pins 18/19, external pullups, 400kHz
    Wire.begin(I2C_SLAVE, 0x66, I2C_PINS_7_8, I2C_PULLUP_EXT, 400000);
    delay(1000);  // wait 1 sec for i2c to start before main loop
    
    // Write shiftreg
    shiftRegisterWriteWord(0x0001);
    
    // something strange with timing
    //pinMode(PI_GPIO18_PIN12, OUTPUT);
    //pinMode(PI_RUN_RESET,OUTPUT);

    // Test Feather / Soft Reboot
    // softReboot();

    // Test Hammer / Hard Reboot
    //hardReboot();

}

//------------------------------------------------------------------------------------------------------------
// Main Loop
//------------------------------------------------------------------------------------------------------------
void loop() {
  //------------------------------------------------------------------------------
  // timeStampMostRecentRx = Stamped by rx event hander when an rx event occurs
  // timeDeltaRx = time since last rx
  // rebootWaitTimeMillis: # of ms to wait for RPi to reboot
  // maxNumSoftRebootTries: hard reset this # of softReboot tries exceeded
  // softRebootTryNum: iterator
  // softReboot(void)
  // hardReboot(void)
  //------------------------------------------------------------------------------

  //------------------------------------------------------------------------------
  // REBOOT CODE TO RESOLVE I2C FAILURE
  //------------------------------------------------------------------------------
  // This block ensures that Teensy is receving i2c data from RPi
  // Cases: (1) rx data has been received w/i 1s
  //            1.1 do nothing
  //        (2) rx data has Not been received in last sec
  //            2.1 soft reboot (up to maxNumSoftRebootTries)
  //            2.2 hard reboot if soft reboot tries exceeds maxNumSoftRebootTries
  //------------------------------------------------------------------------------
  timeDeltaRx = millis()-timeStampMostRecentRx;
  Serial.print("Main Loop: timeDeltaRx = ");
  Serial.println(timeDeltaRx);  

  if(timeDeltaRx > 1000 && softRebootTryNum < maxNumSoftRebootTries)
  {
    Serial.println("******************************");
    Serial.print("In Soft Reboot Loop ");
    Serial.print("Try # ");
    Serial.print(softRebootTryNum+1);
    Serial.print("/");
    Serial.println(maxNumSoftRebootTries);
    Serial.println("******************************");
    
    softReboot();
    Serial.print("Waiting ");
    Serial.print(rebootWaitTimeMillis/1000);
    Serial.println("sec for RPi to boot");
    delay(rebootWaitTimeMillis);                
    Serial.print(rebootWaitTimeMillis/1000);    // By then an rx event should have occured 
    Serial.println(" seconds has elapsed");
    softRebootTryNum +=1;   
  } 
  
  if(timeDeltaRx > 1000 && softRebootTryNum == maxNumSoftRebootTries)
  {                                                  
    hardReboot();
    delay(2000);          // long enough for PI to reboot
    softRebootTryNum = 0; // does it matter that this is after delay(45s)?      
    Serial.print("Waiting ");
    Serial.print(rebootWaitTimeMillis/1000);
    Serial.println("sec for RPi to boot");
    delay(rebootWaitTimeMillis);
    Serial.print(rebootWaitTimeMillis/1000);    // By then an rx event should have occured 
    Serial.println(" seconds has elapsed");
    
  }
  //--------------------------------------------------------------------------
  
  if (ignoreLastRx==0)
  {
    if(received)
    {
      Serial.println("***Received Data***");
      //--------------------------------------------------------
      // Determine mode of operation then execute accordingly
      // PWMSET    = 1     
      // PLC       = 2
      // OPOL      = 3
      // AIN_DRIVE = 4
      // HVDIMMODE = 8
      //--------------------------------------------------------
      if (databuf[0]==PWMSET)
      {
        Serial.println("PWM");
        Serial.print("PWM data = ");
        int i;
        for (i=0;i<(received/2);i++)
        {
          Serial.print("i = ");
          Serial.print(i);
          Serial.print(" data = ");
          int pwmValue=databuf[(2*i)+1]*256+databuf[(2*i)+2];
          Serial.println(pwmValue);
          pwmData[i]=pwmValue;
          analogWrite(pwmPin[i],pwmValue);
        }
        if (LV_HV == 1) //HV mode
        {
          relayUpdate();
        }
      }
      else if (databuf[0]==PLC)
      {
        Serial.print("PLC\n");
        uint8_t plcData=databuf[1];
        Serial.print("PLC data= ");
        Serial.print(plcData);
        Serial.print("\n\n");
        for (int i=0;i<4;i++)
        {
          int state;
          if ((plcData&0x01)>0)
          {
            state=HIGH;
          }
          else
          {
            state=LOW;
          }
          digitalWrite(pSource[i],state);
          plcData/=2;
          
        }
      }
      else if (databuf[0]==OPOL)
      {
        if (LV_HV==0)
        {
          Serial.print("Output Polarity\n");
          uint8_t opolData=databuf[1];
          Serial.print("Output Polarity data= ");
          Serial.print(opolData);
          Serial.print("\n\n");
          shiftRegisterWriteByte(opolData);
        }
        else //High-voltage mode
        {
          int oldDimmerEdge=dimmerEdge;
          dimmerEdge=databuf[1];
          int newDimmerEdge=dimmerEdge;
          Serial.print("Last dimmer edge = ");
          Serial.println(oldDimmerEdge,HEX);
          Serial.print("New dimmer edge = ");
          Serial.println(newDimmerEdge,HEX);
          int i;
          for (i=0;i<8;i++)
          {
            if ((newDimmerEdge&0x00000001) != (oldDimmerEdge&0x00000001))
            {
              Serial.print("Dimmer ");
              Serial.print(i);
              Serial.println(" changed!");
              set0_10V(i,0);
              delay(100);
              set0_10V(i,1);
            }
            newDimmerEdge>>=1;
            oldDimmerEdge>>=1;
          }
        }
      }
      else if (databuf[0]==AIN_DRIVE)
      {
        Serial.println("0-10V Drive");
        Serial.print("0-10V Drive data = ");
        int i;
        for (i=0;i<received;i++)
        {
          Serial.print("i = ");
          Serial.print(i);
          Serial.print(" data = ");
          int driveValue=databuf[i+1]*256;
          Serial.println(driveValue);
          analogWrite(ainDrive[i],driveValue);
        }
      }
      else if (databuf[0]==HVDIMMODE)
      {
        hvDimMode(databuf[1]);
        Serial.println("HVDIMMODE");
      }
    } // End if(ignoreLastRx==0)
    received = 0;
    Serial.println("Received = 0");
  }
} // end loop()


//================================================================================
// Define Helper Functions
//================================================================================

void receiveEvent(size_t count)     
// ------------------------------------------
// count unsigned integer type of atleast 16-bit
// handle Rx Event (incoming I2C data)
// ReceivedAnyI2cFromPi - setup as false. Set to true when first i2c starts
// timeStampMostRecentRx = 0
// ------------------------------------------
{                                   
    // ------------------------------------------------------
    // Debugging for packet reception
    // ------------------------------------------------------
    if (ReceivedAnyI2cFromPi == false) // 1st packet received   
    {
      nTimeStampMostRecentRx = millis();            // If we get an rx it become the most recent
      nTimeStampPriorRx = nTimeStampMostRecentRx;   // no delta yet
      ReceivedAnyI2cFromPi = true;
//      Serial.println("First i2c packet received!");      
    }
    else // > 1st packet received
    {
      nTimeStampPriorRx = nTimeStampMostRecentRx;  // last rx = prior
      nTimeStampMostRecentRx = millis();           // this rx = most recent
      nTimeDeltaRx = nTimeStampMostRecentRx - nTimeStampPriorRx;
      if (nTimeDeltaRx > 10) // filter out false positives
      {
        timeStampMostRecentRx = nTimeStampMostRecentRx; // "passed" to main
//        Serial.println("Got another i2c packet!");
//        Serial.print("Received at t=");
//        Serial.println(timeStampMostRecentRx);
//        Serial.print("Time delta from previous =");
//        Serial.println(nTimeDeltaRx);
        if (nTimeDeltaRx < 1000)
        {
          softRebootTryNum = 0;
//          Serial.print("Resetting softRebootTryNum = ");
//          Serial.println(softRebootTryNum);
        }
      }
    }      
    // *****************************************************************************
 
    size_t idx=0;
    if (count>1)    // If data has already been received
    {
      while(idx < count)
      {
        if(idx < MEM_LEN)                     // drop data beyond mem boundary
            databuf[idx++] = Wire.readByte(); // copy data to mem
        else
            Wire.readByte();                  // drop data if mem full
      }
      Serial.print("Receive Event byte count = ");
      Serial.println(count);
      Serial.print("Data= ");
      for (int i=0;i<16;i++)
      {
        Serial.print(databuf[i]);
        Serial.print(" ");
      }
      Serial.println();
      received = count;
    }
    else      // If data has not been received
    {
      addr=Wire.readByte();
//      Serial.print("Requested to send from address ");
//      Serial.println(addr);
    }
} // end receive event handler

//
// handle Tx Event (outgoing I2C data)
//
void requestEvent(void)
{
  if (addr==VERSION)
  {
    databuf[0]=3;
    databuf[1]=LV_HV;
    databuf[2]=RELEASENUMBERMAJOR;
    databuf[3]=RELEASENUMBERMINOR;
  }
  if (addr==AIN)
  {
//    Serial.println("AIN");
    databuf[0]=8;
    for (int i=0;i<4;i++)
    {
      int data=analogRead(analogIn[i]);
      databuf[(2*i)+1]=char(data/256);
      databuf[(2*i)+2]=char(data&0xFF);
    }
  }
  if (addr==DC)
  {
    int result=0;
    for (int i=3;i>=0;i--)
    {
      result*=2;
      if (digitalRead(dc[i])==HIGH) result+=1;
    }
    databuf[0]=1;
    databuf[1]=result;
  }
  if (addr==IMEAS)
  {
    databuf[0]=16;
    int count=0;
    long data[8]={0,0,0,0,0,0,0,0};
    elapsedMicros t=0;
    while (t<5000)
    {
      for (int i=0;i<8;i++)
      {
        digitalWrite(1,HIGH);
        data[i]+=analogRead(iMeas[i]);
        digitalWrite(1,LOW);
      }
      count++;
    }
//    Serial.println(count);
    for (int i=0;i<8;i++)
    {
//      data[i]*=256;
      data[i]/=count;
      databuf[(2*i)+1]=char(data[i]/256);
      databuf[(2*i)+2]=char(data[i]&0xFF);
    }
  }
//  if (addr==VIN)
//  {
//    databuf[0]=2;
    
//    int data=analogRead(vinSense);
//    databuf[1]=char(data/256);
//    databuf[2]=char(data&0xFF);
//  }
  Wire.write(databuf, MEM_LEN); // fill Tx buffer (send full mem)
}

void setDimmerEdge(int dimmer,int edge)
{
  Serial.print("Set dimmer edge dimmer =  ");
  Serial.print(dimmer);
  Serial.print("edge = ");
  Serial.println(edge);
  if (edge==0)
  {
    dimmerEdge &= ~(0x01<<dimmer);
  }
  else
  {
    dimmerEdge |= 0x01<<dimmer;
  }
  relayUpdate();
}

void set0_10V(int dimmer, int mode)
{
  Serial.print("Set 0_10V mode dimmer =  ");
  Serial.print(dimmer);
  Serial.print(" mode = ");
  Serial.println(mode);
  if (mode==0)
  {
    dimmer0_10Vmode &= ~(0x01<<dimmer);
  }
  else
  {
    dimmer0_10Vmode |= 0x01<<dimmer;
  }
  relayUpdate();
}

void updateHVSR(void)
{
//  int outputWord = ((dimmerEdge&0xF0)<<8)+((dimmerRelayState&0xF0)<<4)+((dimmerEdge&0x0F)<<4)+(dimmerRelayState&0x0F);
  int outputWord = ((dimmerEdge&0x0F)<<12)+((dimmerRelayState&0x0F)<<8)+((dimmerEdge&0xF0))+((dimmerRelayState&0xF0)>>4);
  Serial.print("Update HVSR with ");
  Serial.println(outputWord,HEX);
  shiftRegisterWriteWord(outputWord);
}

void hvDimMode (uint8_t data)
{
  Serial.println("HV Dim Mode");
  Serial.print("HV Dim Mode data = ");
  Serial.println(data,HEX);
  dimmer0_10Vmode=data;
}

void relayUpdate(void)
{
  int i;
  int8_t mask = 0x01;
  Serial.println("Relay Update");
  for (i=0;i<8;i++)  //Don't want to update all the relays at once!
  {
    bool inDimMode=((dimmer0_10Vmode&mask)!=0);
    bool pwmIsZero=(pwmData[i]==0x0000);
    Serial.print("If statement = ");
    Serial.print(dimmerRelayState,HEX);
    Serial.print(inDimMode);
    Serial.print(pwmIsZero);
    if (inDimMode || pwmIsZero) //if channel in (phase-dim-mode) or (PWM is zero) then we need to open the relay.
    {
      if ((dimmerRelayState&mask)==0) //The relay was previously closed, we need to open it
      {
        dimmerRelayState|=mask;
        updateHVSR();
        delay(100);
      }
    }
    else
    {
      if ((dimmerRelayState&mask)!=0) //The relay was previously open, we need to close it
      {
        dimmerRelayState&=~mask;
        updateHVSR();
        delay(100);
      }      
    }
    mask <<= 1;
  }
}

void softReboot(void)
// ------------------------------------------
// Pull RPi pin12/GPIO-18 High for 1 sec 
// RPi pin12 has internal pulldown enabled
// Informs RPi to run sudo shutdown -r now
// ------------------------------------------
{   delay(1000);
    Serial.println("Soft Reboot RPi!");
    Serial.println("PI_GPIO18_PIN12 toggle for 1 sec");
    digitalWrite(PI_GPIO18_PIN12, HIGH);                 
    delay(1000);
    Serial.println("Done");          
    Serial.println("PI_GPIO18_PIN12 returning to normal");
    digitalWrite(PI_GPIO18_PIN12, LOW);       
    Serial.println("Done");           
}

void hardReboot(void)
// ------------------------------------------------
// Pull RPi Run low for 1 sec to reset/reboot
// Teensy High connnects RPi Run low through Mosfet
// ------------------------------------------------
{
    Serial.println("*****************************");
    Serial.println("Hard Reboot RPi!");
    Serial.println("*****************************");
    Serial.println("PI_RUN toggle for 1 sec");
    digitalWrite(PI_RUN_RESET, HIGH);
    delay(1000);
    Serial.println("Done");      
    Serial.println("PI_RUN returning to normal");
    
    digitalWrite(PI_RUN_RESET, LOW);
    Serial.println("Done");
}

//void rebootResolveI2C(void)
//{
//  //------------------------------------------------------------------------------
//  // REBOOT CODE TO RESOLVE I2C FAILURE
//  // Consider changing to accept maxNumSoftRebootTries passed as a parameter 
//  // This block ensures that Teensy is receving i2c data from RPi
//  // Cases: (1) rx data has been received w/i last 1sec
//  //            1.1 do nothing
//  //        (2) rx data has Not been received in last sec
//  //            2.1 soft reboot (up to maxNumSoftRebootTries)
//  //            2.2 hard reboot if soft reboot tries exceeds maxNumSoftRebootTries
//  // softRebootTryNum 
//  // maxNumSoftRebootTries
//  // softReboot(void)
//  // hardReboot(void)
//  //------------------------------------------------------------------------------
//  // int softRebootTryNum = 0;
//  
//  while(softRebootTryNum < maxNumSoftRebootTries && timeSinceLastRx > 1000)     //
//  {
//    softReboot();
//    maxNumSoftRebootTries += 1;
//    delay(rebootWaitTimeMillis); 
//    currentMillis = millis();
//   }                                                  // 
//    hardReboot();                                     // 
//    softRebootTryNum = 0;
//    delay(rebootWaitTimeMillis);
//}
