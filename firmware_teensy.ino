//-------------------------------------------------------------------------------------
// Revolution Lighting Project Firmware
//-------------------------------------------------------------------------------------
// Version 0.5
// (1) fixed bug in AIN_Drive set command which caused dry contact #1 input to be
//     corrupted, i.e. stuck on logic 1
// (2) This version used for Amazon demo
//
// Version 0.4
// (1) resolved wet/dry contact failure on all
// (2) found sizeof(array) bug and implemented work around
//
// Version 0.3
// -resolved 0-10v current sensing error
// -added flag to disable/enable failsafe reboot code
// -sped up teensy boot time removing 45s delay from setup()
// (1) bugfix for analogIn pins (pinMode(xx, INPUT) for all analog in pins
// (2) changed AnalogWriteFrequency to 10KHz to resolve current sense read error
// (3) brokeout failsafe reboot code into a helper function rebootResolveI2C()
// (4) removed 45s delay from setup()
// (5) reimplemented 45s delay as a conditional in main loop
// (6) added EnableFailSafeReboot boolean to enable or disable failsafe reboot code
//
// Version 0.2
// -this version used for WW22 demos in Austin... and other locations
// -changes to phase dimming and 0-10V mode
// (1) resolved reverse logic error in hvDimMode() -> resolve w/Nick next version 
// (2) added serial debugging to relayUpdate()
// (2) removed superfluous helper functions 
// (3) more commenting, started refactoring for standard 80 character width
// (4) commented out failsafe reboot code but did not remove it - not req'd WW22 demos
//
// Version 0.1
// -Initial commit github.com 
// -feature ads and bug fixes from non-git 0.1
// (1) resolved timeout/crash issue after mode set
// (2) implemented/tested soft reboot (RPi Pin12/GPIO18 toggle)
// (3) implemented/tested hard reboot (RPi RUN toggle)
// (4) implemented/tested fail safe (soft/feather and hard/hammer reboot logic)
// (5) implemented logic to ignore duplicate node gets
// (6) resolved event handler crashes
// (7) started commenting/refactoring code for readability
//-------------------------------------------------------------------------------------
#define RELEASENUMBERMAJOR 0
#define RELEASENUMBERMINOR 6
#define LV_HV 0 //LV=0, HV=1
#include <i2c_t3.h>

//-------------------------------------------------------------------------------------
//  # of elements in array = total array length / size of each element
//-------------------------------------------------------------------------------------
#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

//-------------------------------------------------------------------------------------
// Function prototypes for event handlers
//-------------------------------------------------------------------------------------
void receiveEvent(size_t count);  // I2C Receive event handler (count = numBytesRxd)
void requestEvent(void);          // I2C Request data event handler


//-------------------------------------------------------------------------------------
// Memory
//-------------------------------------------------------------------------------------
#define MEM_LEN 256   //
#define VERSION 0     // get command code for version
#define PWMSET 1      // set command code for PWM (channel and value 0-65535)
#define PLC 2         // get command code for
#define OPOL 3        // 
#define AIN_DRIVE 4   //
#define AIN 5         //
#define DRY_CONTACT 6 // get command code for wet/dry contact
#define IMEAS 7       //
#define HVDIMMODE 8   //
#define VIN 9         //
#define SR_CLK 19     // I2c clock
#define SR_DAT 17     // I2C data
#define SR_LAT 18     // ShiftReg clock

//-------------------------------------------------------------------------------------
// Globals for Teensy-to-Pi Rest Interface
//-------------------------------------------------------------------------------------

boolean EnableFailSafeReboot = false;                   // false disables failsafe soft/hard reboot code
boolean ReceivedAnyI2cFromPi = false;                   // Set true 1st time Teensy receives data from Pi
boolean bootTimeWaitDone = false;                       // Set true in mainloop after bootWaitTime exceeded
unsigned long bootUpMillis;                             // timestamp when setup() ran
unsigned long rebootWaitTimeMillis = 45000;             // 45 seconds
unsigned long timeStampMostRecentRx;                    // loop(): Zero indicates no Rx event received yet
unsigned long timeSinceMostRecentRx;                    // loop(): Zero indicates no Rx event received yet
unsigned long timeDeltaRx = 0;                          // loop(): 

int softRebootTryNum = 0;                               // iterator
int maxNumSoftRebootTries = 3;                          // max # soft reboot tries before hard rebooting

// Used in received event handler only
unsigned long nTimeStampMostRecentRx;                   // Used in rx event handler
unsigned long nTimeDeltaRx = 0;                         // ""
unsigned long nTimeStampPriorRx;                        // ""

//-------------------------------------------------------------------------------------
// Globals Pin Definitions
//-------------------------------------------------------------------------------------

const int PI_RUN_RESET = 31;                            // Hard reboot: Teensy Pin23 to RPi Run (GPIO-31)
const int PI_GPIO18_PIN12 = 32;                         // Soft reboot: Teensy Pin32 (TP2) to Rpi GPIO-18-pin12 (TP21)

const int pwmPin[] = {2,3,4,5,6,9,10,29};               // pwmPins 
const int n_pwmPin = 8;                                 // # of pwmPins -> reimplement w/STL

const int analogIn[] = {A22,A0,A1,A2};                  // 
const int n_analogIn = 4;                               //

const int pSource[] = {27,28,11,13};                    // 
const int n_pSource =  4;                               //

const int ainDrive[] = {21,22,23,30};                   // 
const int n_ainDrive = 4;                               //

const int dc[] = {12,24,25,26};                         // Dry contact
const int n_dc = 4;                                     //

const int iMeas[] = {A14,A15,A16,A17,A18,A19,A20,A21};  // Current sense
const int n_iMeas = 8;

uint8_t ignoreLastRx=0;                                 // ***
volatile uint8_t received;                              // ***
int dimmer0_10Vmode = 0x00;                             // ***
int dimmerEdge = 0x00;                                  // ***
uint8_t dimmerRelayState = 0x00;                        // ***
uint16_t pwmData[] = {0x0000,0x0000,0x0000,0x0000,      // ***
                      0x0000,0x0000,0x0000,0x0000};



#define QUEUE_SIZE 10
// NGP addons
// que for set commands 
uint8_t queue2[QUEUE_SIZE][200];
int front = 0, rear = 0;
void enQueue(uint8_t * data, int length);
void deQueuebuff(uint8_t * rxdata);

void TxVersionInfo(); // sends out version info, util func. 

// methods for measurements
void MeasureWetDryContactInputs();
void MeasureZero2TenInputs();
void MeasurePWMCurrent();  // take pwm current measurment

// measurment data holders / current status holders
uint8_t pwmcurrentstatus[17]; // pwm current (for power)
uint8_t zero2teninputstatus[9]; // zero to 2 input (analog)
uint8_t wetdrycontactstatus[2]; // wd contact (binary)

// routines for set hw 
void setOutputPolarity(uint8_t mode);
void setZero2TenVoltDrive(uint8_t * drivebuff);
void setPLCState(uint8_t stateval);
void setPWMState(uint8_t * pwmvals);
void setHVDimMode(uint8_t dimmode);


volatile long loopcount = 0;  //debug only 


byte pending_get_command = 0xff;


//void printdebug(char* str, int flag)
//{
//    if(flag)
//        Serial.print(str);
//}
//-------------------------------------------------------------------------------------
// Setup
//-------------------------------------------------------------------------------------
void setup()
{
 
 // Configure Teensy Pins
    pinMode(1,OUTPUT);                // TP1
    pinMode(SR_CLK,OUTPUT);           // I2C clock
    pinMode(SR_DAT,OUTPUT);           // I2C data
    pinMode(SR_LAT,OUTPUT);           // ShiftRegister clock
    pinMode(PI_RUN_RESET,OUTPUT);     // Hard reboot       
    pinMode(PI_GPIO18_PIN12, OUTPUT); // Soft reboot       
    //pinMode(vinSense,INPUT);
  
    int i;
    
    for (i=0;i<(sizeof(pSource)/4);i++) // length of total array / length of element (byte) = # elements
    {
      pinMode(pSource[i],OUTPUT);
      digitalWrite(pSource[i],LOW);
    }
    
    for (i=0;i<(sizeof(pwmPin)/4);i++)
    {
      pinMode(pwmPin[i],OUTPUT);
      analogWriteFrequency(pwmPin[i],200);    // PWM freq 
      analogWrite(pwmPin[i] , 0);
    }

    
//  //Disabled due to stomping    
//    for (i=0;i<(sizeof(dc)/4);i++)
//    {
//      pinMode(dc[i],INPUT);
//    }

    pinMode(dc[0],INPUT);
    pinMode(dc[1],INPUT);
    pinMode(dc[2],INPUT);
    pinMode(dc[3],INPUT);

    for (i=0;i<(sizeof(iMeas)/4);i++)
    {
      pinMode(iMeas[i],INPUT);
    }
    for (i=0;i<(sizeof(ainDrive)/4);i++)
    {
      pinMode(ainDrive[i],OUTPUT);
      analogWriteFrequency(ainDrive[i],10000);      // 10kHz PWM freq
    } 
    for (i=0;i<(sizeof(analogIn)/4);i++)                // 
    {
      pinMode(analogIn[i],INPUT);
    }
 
    analogWriteResolution(16);
    analogReadResolution(12);
    analogReadAveraging(256);
   
 // Data init
    received = 0;
  
    Serial.begin(115200);
    Serial.println("Teensy Setup has run! We've Booted!");
        
    //delay must take place before interupt handlers are enabled
    //interupt handler ignore delay statementswait 45 seconds
    bootUpMillis = millis();
    //Serial.println("Waiting 45 seconds");
    //delay(rebootWaitTimeMillis);
    //Serial.println("45 seconds since Teensy booted");
    delay(1000);

 // Register wire/I2c slave received and request handlers
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

 // Setup for Slave mode, address 0x66, pins 18/19, external pullups, 400kHz
    Wire.begin(I2C_SLAVE, 0x66, I2C_PINS_7_8, I2C_PULLUP_EXT, 400000);
    delay(1000);  // wait 1 sec for i2c to start before main loop
    
 // Write shiftreg
    shiftRegisterWriteWord(0x0001);
   
  // Test Feather / Soft Reboot
     //softReboot();

  // Test Hammer / Hard Reboot
     //hardReboot();
     Serial.println("loop start");
}

void loop()
//-------------------------------------------------------------------------------------
// Main Loop
//-------------------------------------------------------------------------------------
// EnableFailSafeReboot: true to enable failsafe reboot code, false to disable
// timeStampMostRecentRx = Stamped by rx event hander when an rx event occurs
// timeDeltaRx = time since last rx
// rebootWaitTimeMillis: # of ms to wait for RPi to reboot
// maxNumSoftRebootTries: hard reset when this # of softReboot tries exceeded
// softRebootTryNum: iterator
// softReboot(void)
// hardReboot(void)
// rebootResolveI2C(void): failsafe reboot mode
//-------------------------------------------------------------------------------------
{
     uint8_t rxdata[200];      // used for pulling in latest from queue. 
           
     memset(rxdata, 0, sizeof(rxdata));  // /4 
  
      loopcount++;
      if(loopcount == 200)
      {
         loopcount = 0;
        // Serial.println("loop rollover");
      }

      deQueuebuff(rxdata);  // dequeue from set que,  is anything incomming, 

       // if valid data from queue,  process it, 
       // *********************************************************
      if(rxdata[0] != 0xff)
      {
          // Serial.print("valid data ");
          //Serial.println(rxdata[0]);
          switch(rxdata[0])
          {
             case 0x01:
                setPWMState(rxdata);                 
             break;
             case 0x02:
               setPLCState(rxdata[1]);
               break;
             case 0x03:
               setOutputPolarity(rxdata[1]);
             break;
             case 0x04:
               setZero2TenVoltDrive(rxdata);
               break;
             case 0x08:
                setHVDimMode(rxdata[1]);
               break;
            default:
            break;
          }// end switch 
      }
      else
      {
        // Serial.println("nothing in set queue");
        // do nothing ,  nothing to process from i2c 
      }


     // measurement routines 
     MeasurePWMCurrent();
     MeasureZero2TenInputs();
     MeasureWetDryContactInputs();
   
} // end main loop()

//-------------------------------------------------------------------------------------
// Define Helper Functions
//-------------------------------------------------------------------------------------
// ------------------------------------------
// Node set command received:
// handle Rx Event (incoming I2C data)
// count = number of bytes received
// ReceivedAnyI2cFromPi - setup as false. Set to true when first i2c starts
// timeStampMostRecentRx = 0
// ------------------------------------------
void receiveEvent(size_t count)     
{      
      uint8_t rxBuff[50];
      
      for (int i=0;i<count;i++)
      {
        rxBuff[i] = Wire.readByte(); // copy data to mem
      }
   
      // for get commands don't put into que,  just process straight
      if(rxBuff[0] == IMEAS || rxBuff[0] == AIN || 
      rxBuff[0] == DRY_CONTACT || rxBuff[0]==VERSION)
      {
         pending_get_command = rxBuff[0];
      }
      else      
         enQueue(rxBuff, count);  // queue as set command 
    
} // end receive event handler



void requestEvent(void)
// ------------------------------------------
// Node GET Command received:
// Teensy Tx Event (I2C data -> PI)
// addr = command byte requested by Node i2c
// ------------------------------------------
{
 
  
  if (pending_get_command==VERSION)              // 0 - 
      TxVersionInfo();
      
  if(pending_get_command == IMEAS)
      Wire.write(pwmcurrentstatus, 17);

  if(pending_get_command == AIN)
  {
     // Serial.println("got zero 2 ten request");
      Wire.write(zero2teninputstatus, 9);
  }

  if(pending_get_command == DRY_CONTACT)
  {
      // Serial.println("got wd request");
      Wire.write(wetdrycontactstatus, 2);
  }

  pending_get_command = 0xff; // clear it 

/*  still to do for hv board ----------------------------------------<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< to do . 
//  if (addr==VIN)                // 9
//  {
//    databuf[0]=2;
    
//    int data=analogRead(vinSense);
//    databuf[1]=char(data/256);
//    databuf[2]=char(data&0xFF);
//  }
  Wire.write(databuf, MEM_LEN); // fill Tx buffer (send full mem)
    */
  
}

void setDimmerEdge(int dimmer,int edge)
// ------------------------------------------
// dimmer = 
// edge = 
// ------------------------------------------
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
// ------------------------------------------
// dimmer
// dimmer0_10Vmode
// mode
// relayUpdate()
// ------------------------------------------
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
// ------------------------------------------
// dimmerEdge
// dimmerRelayState
// outputWord
// shiftRegisterWriteWord()
// ------------------------------------------
{
  int outputWord = ((dimmerEdge&0xF0)<<8)+((dimmerRelayState&0xF0)<<4)+((dimmerEdge&0x0F)<<4)+(dimmerRelayState&0x0F);
//  int outputWord = ((dimmerEdge&0x0F)<<12)+((dimmerRelayState&0x0F)<<8)+((dimmerEdge&0xF0))+((dimmerRelayState&0xF0)>>4);
  Serial.print("Update HVSR with ");
  Serial.println(outputWord,HEX);
  shiftRegisterWriteWord(outputWord);
}

void hvDimMode (uint8_t data)
// ------------------------------------------
// dimmer0_10Vmode
// ------------------------------------------
{
  Serial.println("HV Dim Mode");
  Serial.print("HV Dim Mode data = ");
  Serial.println(data,HEX);
  dimmer0_10Vmode=~data; //Fixme with Nick, logic inversion ... see ~
}

void relayUpdate(void)
// ------------------------------------------
// ******
// dimmerRelayState
// inDimMode
// mask
// pwmIsZero
// updateHVSR()
// ------------------------------------------
{
  int i;
  int8_t mask = 0x01;
  Serial.println("Relay Update");
  for (i=0;i<8;i++)  //Don't want to update all the relays at once!
  {
    bool inDimMode=((dimmer0_10Vmode&mask)!=0);
    bool pwmIsZero=(pwmData[i]==0x0000);
    //Serial.print("If statement = ");
    Serial.print("dimmerRelayState= ");
    Serial.print(dimmerRelayState,HEX);
    Serial.print(" inDimMode=");
    Serial.print(inDimMode);
    Serial.print(" pwmIsZero=");
    Serial.println(pwmIsZero);
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

void rebootResolveI2C(void)
//------------------------------------------------------------------------------
// FAILSAFE MODE - REBOOT CODE TO RESOLVE I2C FAILURE
//------------------------------------------------------------------------------
// This block ensures that Teensy is receving i2c data from RPi
// Cases: (1) rx data has been received w/i 1s
//            1.1 do nothing
//        (2) rx data has Not been received in last sec
//            2.1 soft reboot (up to maxNumSoftRebootTries)
//            2.2 hard reboot if soft reboot tries exceeds maxNumSoftRebootTries
//------------------------------------------------------------------------------
{
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
}


void enQueue(uint8_t * data, int length){
  
    // Serial.print("EnQUE ");
   /* if(data[0] == 1)
    {
         Serial.println("EnQUE ");
        for(int j = 0 ; j < 9 ;j++)
        {
            Serial.print(data[j]);
            Serial.print(" | ");
        }
    }  */
       
     memcpy(queue2[rear], data, length); 
   
     rear++;
     if(rear >= QUEUE_SIZE-1)
       rear = 0;


    //if(data[0] == 1)
      //    printQueue();
     // Serial.print(front);
}


void printQueue()
{
     for(int i = 0 ; i < 10; i++)
     {
           Serial.print(i);
           Serial.print(" --- ");
            for(int j = 0 ; j < 20 ;j++)
        {
            Serial.print(queue2[i][j]);
            Serial.print(" | ");
        }
        Serial.println(" ");
        
     }
}

void deQueuebuff(uint8_t * rxdata){

   if(front == rear)
   {
      rxdata[0] = 0xff;
      return; 
   }
   else{

   // Serial.print("DQ  ");  
   // Serial.println(front);
   // Serial.println(rear);
      
     // Serial.print("got valid buffer data"); 
     // Serial.println(front);
      
      memcpy(rxdata, queue2[front], 200);   

      memset(queue2[front],0xFF,200);
       
      front++;
      if(front >= QUEUE_SIZE-1)
          front = 0;
    

      return; 
   }
}


void MeasureWetDryContactInputs()
{
    int result=0;
    for (int i=3;i>=0;i--)
    {
      result*=2;  // left shift 1
      if (digitalRead(dc[i])==HIGH) result+=1;
    }
    wetdrycontactstatus[0]=1;   //length
    wetdrycontactstatus[1]=result;     
}

void TxVersionInfo()
{
    uint8_t txbuff[4];
    Serial.println("got version request, sending now" );
    txbuff[0]=3;
    txbuff[1]=LV_HV;
    txbuff[2]=RELEASENUMBERMAJOR;
    txbuff[3]=RELEASENUMBERMINOR;
    Wire.write(txbuff, 4);
}


void MeasureZero2TenInputs()
{
  int tempdata = 0;
   memset(zero2teninputstatus,0,sizeof(zero2teninputstatus));
    zero2teninputstatus[0]=8;

    tempdata=analogRead(analogIn[0]);            
    zero2teninputstatus[1]=(uint8_t)(tempdata >> 8)&0xFF;
    zero2teninputstatus[2]=(uint8_t)(tempdata&0xFF);
   
    tempdata=analogRead(analogIn[1]);
    zero2teninputstatus[3]=(uint8_t)(tempdata >> 8)&0xFF;
    zero2teninputstatus[4]=(uint8_t)tempdata&0xFF;
    tempdata=analogRead(analogIn[2]);     
    zero2teninputstatus[5]=(uint8_t)(tempdata >> 8)&0xFF;
    zero2teninputstatus[6]=(uint8_t)(tempdata&0xFF);
     
    tempdata=analogRead(analogIn[3]);
    zero2teninputstatus[7]=(uint8_t)(tempdata >> 8)&0xFF;
    zero2teninputstatus[8]=(uint8_t)(tempdata&0xFF);
}


void MeasurePWMCurrent()
{
 long data[8]={0,0,0,0,0,0,0,0};  
   int count = 0; 
   
    elapsedMicros t=0;
    while (t<5000)
    {
        for (int i=0;i<8;i++)
        {
          //digitalWrite(1,HIGH);
          data[i]+=analogRead(iMeas[i]);
          //digitalWrite(1,LOW);
        }
        count++; //for div
    }  

    memset(pwmcurrentstatus,0,sizeof(pwmcurrentstatus));
    pwmcurrentstatus[0]=16;
    for (int i=0;i<8;i++)
    { 
      data[i]/=count;
      pwmcurrentstatus[(2*i)+1]=char(data[i]/256);
      pwmcurrentstatus[(2*i)+2]=char(data[i]&0xFF);
    }
}


void setOutputPolarity(uint8_t mode)
{
   if (LV_HV==0)
    {
      Serial.print("Output Polarity\n");
      uint8_t opolData=mode;
      Serial.print("Output Polarity data= ");
      Serial.print(opolData);
      Serial.print("\n\n");
      shiftRegisterWriteByte(opolData);
    }
    else //High-voltage mode
    {
      int oldDimmerEdge=dimmerEdge;
      dimmerEdge=mode;
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



void setZero2TenVoltDrive(uint8_t * drivebuff)
{

   // Serial.println("0-10V Drive");
   // Serial.print("0-10V Drive data = ");
    int i;
    int numAin=received-1; //numAin is the number of analog inputs to update.  We don't have to do all of them...
    if ((received-1)>(sizeof(ainDrive)/4)) //...but trying to update more than we have is bad.
    {
      Serial.println("We received too many bytes.  Smack Nick.");
      numAin=(sizeof(ainDrive)/4);
    }
    
    for (i=0;i<numAin;i++) //Received count is equal to the command plus all of its arguments.  "received-1" strips the command byte
    {
    //  Serial.print("i = ");
    //  Serial.print(i);
    //  Serial.print(" data = ");
      int driveValue=drivebuff[i+1]*256;
    //  Serial.println(driveValue);
      analogWrite(ainDrive[i],driveValue); 
   }
}
void setPLCState(uint8_t stateval)
{
  //  Serial.print("PLC data= ");
   // Serial.println(stateval);
    for (int i=0;i<4;i++)
    {
        if(  ( (stateval >> i) & 0x01) > 0)
            digitalWrite(pSource[i],HIGH);
        else
            digitalWrite(pSource[i],LOW);
    }
}

void setPWMState(uint8_t * pwmvals)
{
  // Serial.print("pwmvals: ");
   for(int i = 0; i < 8; i++)
   {
      int pwmValue=pwmvals[(2*i)+1]*256+pwmvals[(2*i)+2];
      int pct = (pwmValue*100)/65535;
    //  Serial.print(pwmValue);
   //    Serial.print(pct);
    //  Serial.print(" | ");
    
      pwmData[i]=pwmValue; // store value for ref 
      analogWrite(pwmPin[i],pwmValue);
   }

  // Serial.println("");
}

void setHVDimMode(uint8_t dimmode)
{
      Serial.println("HVDIMMODE");  
      hvDimMode(dimmode);  
}


