//-------------------------------------------------------------------------------------
// Revolution Lighting Project Firmware
//-------------------------------------------------------------------------------------
// Version 0.9
// Changed ain freq from 10K to 200 hz becuase of shared timer  issue ftm0/1,2,3... between 
// pwm outs, 

// Version 0.8
// Added function prototypes to aid in porting to .cpp
// RMS Current measurement function implemented
// RMS Current main loop function call logic implemented
// Verified to compile and upload with Atmel Studio 7 w/Arduino plugin and Arduinio IDE
//
// Version 0.7
// Code reworked to accommodate RL0005 Rev.B and RL0003 Rev.C
// shiftRegister code and associated calls reworked to accommodate new HW
//
// Version 0.6
// Nick's FIFO Queue implemented for get commands
// Set command implementations broken out into separate functions
// Serial.print code updated
//
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
// (1) bug fix for analogIn pins (pinMode(xx, INPUT) for all analog in pins
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
#define RELEASENUMBERMINOR 9
#define LV_HV 0 //LV=0, HV=1
#include <arduino.h>
#include <i2c_t3.h>

boolean DebugRMSCurrent = true;
boolean TestFlag = true;

//-------------------------------------------------------------------------------------
//  # of elements in array = total array length / size of each element
//-------------------------------------------------------------------------------------
#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

//-------------------------------------------------------------------------------------
// Function prototypes
//-------------------------------------------------------------------------------------
void receiveEvent(size_t count);
void requestEvent(void);
void setup(void);
void loop(void);
void setDimmerEdge(int dimmer, int edge);
void set0_10V(int dimmer, int mode);
void updateHVSR(int dimmeredgle, uint8_t dimmerrelaystate);
void hvDimMode (uint8_t data);
void relayUpdate(void);
void softReboot(void);
void hardReboot(void);
void rebootResolveI2C(void);
void enQueue(uint8_t * data, int length);
void printQueue(void);
void deQueuebuff(uint8_t * rxdata);
void TxVersionInfo(void);  // sends out version info, util func.

// methods for measurements
void MeasureWetDryContactInputs(void);
void MeasureZero2TenInputs(void);
void MeasurePWMCurrent(void);   // take pwm current measurment
void MeasureRMSCurrent(int chNum);  //

// routines for set hw
void setOutputPolarity(uint8_t mode);
void setZero2TenVoltDrive(uint8_t * drivebuff);
void setPLCState(uint8_t stateval);
void setPWMState(uint8_t * pwmvals);
//void setHVDimMode(uint8_t dimmode);

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
#define SR_CLK 19     // sr clock
#define SR_DAT 17     // sr data out line
#define SR_LAT 18     // sr latch

#define SR_DAT_IN 20     // I2C data in, 
// 20 for input read

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

const int pwmPin[] = {2, 3, 4, 5, 6, 9, 10, 29};        // pwmPins
const int n_pwmPin = 8;                                 // # of pwmPins -> reimplement w/STL

const int analogIn[] = {A22, A0, A1, A2};               //
const int n_analogIn = 4;                               //

const int pSource[] = {27, 28, 11, 13};                 //
const int n_pSource =  4;                               //

const int ainDrive[] = {21, 22, 23, 30};                //
const int n_ainDrive = 4;                               //

const int dc[] = {12, 24, 25, 26};                      // Dry contact
const int n_dc = 4;                                     //

const int iMeas[] = {A14, A15, A16, A17, A18, A19, A20, A21}; // Current sense
const int n_iMeas = 8;

uint8_t ignoreLastRx = 0;                               // ***
volatile uint8_t received;                              // ***
int hvDimModeMask = 0x00;                             // ***
int dimmerEdge = 0x00;                                  // ***
uint8_t dimmerRelayState = 0x00;                        // ***
uint16_t pwmData[] = {0x0000, 0x0000, 0x0000, 0x0000,   // ***
                      0x0000, 0x0000, 0x0000, 0x0000
                     };
//-------------------------------------------------------------------------------------
// *** RMS CURRENT Globals
//-------------------------------------------------------------------------------------
// configure current measurement details here:
unsigned long samplesPerSec = 1000;                               //
unsigned long windowLenSecs = 1;                                  // 1 second window len

// Calculate sampling time period in mS => 1/samplesPerSec*1000
// unsigned long iSamplePeriod = (1/samplesPerSec)*1000;          // **this is returning 0 for some reason**
unsigned long iSamplePeriod = 1;                                  // so, do manual calculation instead

// Allocate memory for iWindow array
//const int len_iWindow = int(samplesPerSec*windowLenSecs);       // **This isn't casting to type int**
//const int len_iWindow = (int)samplesPerSec*windowLenSecs;       // **same as above...**
const int len_iWindow = 1000;                                     // so, do manual calculation instead
long iWindow[ len_iWindow ];                                      //

// temp variables
long iWindowSum = 0;                    // for removing dc bias
long iWindowAvg = 0;                    // for removing dc bias
long iWindowPow2Sum = 0;                // for rms calc
long iWindowPow2Mean = 0;               // for rms calc
long iRMS = 0;                          // RMS current from window

// sample timestamping
unsigned long iTimeLastSample  = 0;     // time stamp of previous current meas.
// data "returned" via this array
long rmsCurrentArray[8] = {0, 0, 0, 0, 0, 0, 0, 0};   // this array store rms current

//-------------------------------------------------------------------------------------
// *** QUEUE Globals
//-------------------------------------------------------------------------------------
#define QUEUE_SIZE 10
// NGP addons
// que for set commands
uint8_t queue2[QUEUE_SIZE][200];
int front = 0, rear = 0;

// measurement data holders / current status holders
uint8_t pwmcurrentstatus[17]; // pwm current (for power)
uint8_t zero2teninputstatus[9]; // zero to 2 input (analog)
uint8_t wetdrycontactstatus[2]; // wd contact (binary)

volatile long loopcount = 0;  //debug only
byte pending_get_command = 0xff;


// 8/3/17  power failover stuff
uint8_t powerfailovermask = 0x03;   // 0 is emergency ,  1 is utility line.

unsigned long linepower_1_4_failstart = 0x00;  // set when line power first fails. to detect 1 sec of fialure.
unsigned long linepower_5_8_failstart = 0x00;
byte failover_ignorerequests = 0x00;


long chan0rms_meas[200]; // chan 0 holds analog read values (raw)
long chan0avg = 0x00;  // for debug, to fix...
//-------------------------------------------------------------------------------------

void setup()
{
  // Configure Teensy Pins
  pinMode(1, OUTPUT);               // TP1
  pinMode(SR_CLK, OUTPUT);          // I2C clock
  pinMode(SR_DAT, OUTPUT);          // I2C data
  pinMode(SR_DAT_IN, INPUT);          // I2C data
  pinMode(SR_LAT, OUTPUT);          // ShiftRegister clock
  pinMode(PI_RUN_RESET, OUTPUT);    // Hard reboot
  pinMode(PI_GPIO18_PIN12, OUTPUT); // Soft reboot
  //pinMode(vinSense,INPUT);

  int i;

  for (i = 0; i < (sizeof(pSource) / 4); i++) // length of total array / length of element (byte) = # elements
  {
    pinMode(pSource[i], OUTPUT);
    digitalWrite(pSource[i], LOW);
  }

  for (i = 0; i < (sizeof(pwmPin) / 4); i++)
  {
    pinMode(pwmPin[i], OUTPUT);
    
    if (LV_HV == 0)
    {
      analogWriteFrequency(pwmPin[i], 200);   // PWM freq
    }
    else
    {
      Serial.print("setup pwm pin Wrong for this BOARD: ");
      
      analogWriteFrequency(pwmPin[i], 4101);
    }

    analogWrite(pwmPin[i] , 0);
  }

  //  //Disabled due to stomping
  //    for (i=0;i<(sizeof(dc)/4);i++)
  //    {
  //      pinMode(dc[i],INPUT);
  //    }

  pinMode(dc[0], INPUT);
  pinMode(dc[1], INPUT);
  pinMode(dc[2], INPUT);
  pinMode(dc[3], INPUT);

  for (i = 0; i < (sizeof(iMeas) / 4); i++)
  {
    pinMode(iMeas[i], INPUT);
  }
  for (i = 0; i < (sizeof(ainDrive) / 4); i++)
  {
    pinMode(ainDrive[i], OUTPUT);
    analogWriteFrequency(ainDrive[i], 200); //  8/22 changed from 10K, to 200 Hz, to fix pwm channels. 10000);     // 10kHz PWM freq
  }
  for (i = 0; i < (sizeof(analogIn) / 4); i++)        //
  {
    pinMode(analogIn[i], INPUT);
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
  shiftRegisterWriteWord16(0x0001);

  // Test Feather / Soft Reboot
  //softReboot();

  // Test Hammer / Hard Reboot
  //hardReboot();

  // Print RMS current measurement settings:
  //     printdebug("Testing Print Debug\n", DebugRMSCurrent);
  //     printdebug("RMS Current Sampling: Samples/Sec = ", DebugRMSCurrent);
  //     //printdebug(samplesPerSec, DebugRMSCurrent);
  //     printdebug("\n", DebugRMSCurrent);

  /*   Serial.print("Window Len (sec) = ");
     Serial.println(windowLenSecs);
     Serial.println("");

     Serial.print("Data points in iWindow[N] = ");
     Serial.println(len_iWindow);
     Serial.println("");

     Serial.print("Sample Period = ");
     Serial.print(iSamplePeriod);
     Serial.println(" mS");
     Serial.println("");*/

  //     printf("This is my first time using arduino printf %f", 3.14);
  Serial.println("loop start");

}	// End setup()

void loop()
{
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
  uint8_t rxdata[200];      // used for pulling in latest from queue.
  memset(rxdata, 0, sizeof(rxdata));  // /4

  deQueuebuff(rxdata);  // dequeue from set que,  is anything incomming,

  // if valid data from queue,  process it,
  // *********************************************************
  if (rxdata[0] != 0xff)
  {
    Serial.print("valid data received; command byte = ");
    Serial.println(rxdata[0]);
    if (failover_ignorerequests == 0x00)
    {
      switch (rxdata[0])
      {
        case 0x01:
          setPWMState(rxdata);
          break;
        case 0x02:
          setPLCState(rxdata[1]);
          break;
        case 0x03:  // output pol ,  dim direction (hv)
          setOutputPolarity(rxdata[1]);
          break;
        case 0x04:
          setZero2TenVoltDrive(rxdata);
          break;
        case 0x08:
          hvDimMode(rxdata[1]);
          break;
        default:
          break;
      }// end switch
    }
    else
    {
      Serial.println(" command ignored,  in failover mode");
    }
  }
  else
  {
    // Serial.println("nothing in set queue");
    // do nothing ,  nothing to process from i2c
  }

  // measurement routines

  if (LV_HV == 0)
    MeasurePWMCurrent();
  //else
  //{
    // RMS Current Sampling
    // Note: iTimeLastSample intitialized to 0 in setup()
    //if ( (millis() - iTimeLastSample) >= iSamplePeriod ) // time to measure next sample?
    //{
    //  for (int chNum = 0; chNum < 8; chNum++)
    //  {
      // MeasureRMSCurrent(0);    // put scalar rms current value in: rmsCurrentArray[chNum]
     // }
    //}
  //}


  MeasureZero2TenInputs();
  MeasureWetDryContactInputs();

  POEAndLinePowerFailoverHandler();


  loopcount++;
  if (loopcount == 1000)
  {
    loopcount = 0;
    // Serial.println("loop rollover");
  }


  // ************************* TEST CODE FOR FLIPPING RELAYS ****

  // loop test for normal output relays , channels 1- 8
  //if(loopcount == 200)
  //  setdimmerRelayState(5,true);
  // else if(loopcount == 700)
  //   setdimmerRelayState(5,false);


  //if(loopcount == 200)
  // setFailoverRelay(1,true);
  // else if(loopcount == 700)
  //    setFailoverRelay(1,false);

 // if (loopcount == 200)
  //{
   // Serial.print("rms current chan 0:  ");
   // Serial.println(rmsCurrentArray[0]);

 // }
  /*  if(loopcount == 200)
    {
      byte val = shiftRegisterReadByte();
       Serial.print("value read from shiftreg: ");
      Serial.println(val,HEX);
    }

  */



} // end main loop()

void receiveEvent(size_t count)
{
  uint8_t rxBuff[50];

  for (int i = 0; i < count; i++)
  {
    rxBuff[i] = Wire.readByte(); // copy data to mem
  }

  // for get commands don't put into que,  just process straight
  if (rxBuff[0] == IMEAS || rxBuff[0] == AIN ||
      rxBuff[0] == DRY_CONTACT || rxBuff[0] == VERSION)
  {
    pending_get_command = rxBuff[0];
  }
  else
    enQueue(rxBuff, count);  // queue as set command

} // end receive event handler

void requestEvent(void)
{
  // ------------------------------------------
  // Node GET Command received:
  // Teensy Tx Event (I2C data -> PI)
  // addr = command byte requested by Node i2c
  // ------------------------------------------

  if (pending_get_command == VERSION)            // 0 -
    TxVersionInfo();

  if (pending_get_command == IMEAS)
    Wire.write(pwmcurrentstatus, 17);

  if (pending_get_command == AIN)
  {
    // Serial.println("got zero 2 ten request");
    Wire.write(zero2teninputstatus, 9);
  }

  if (pending_get_command == DRY_CONTACT)
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



void setdimmerRelayState(int relaynumber, bool state)  // state 0 (phase)  /1 = 0 -10 volt.
{
  if (state)
  {
    dimmerRelayState |= (0x01 << (relaynumber - 1)); // set bit,
  }
  else
  {
    dimmerRelayState &= ~(0x01 << (relaynumber - 1)); // clear bit,
  }
  Serial.print("dimmerRelayState: ");
  Serial.println(dimmerRelayState, HEX);
  updateHVSR(dimmerEdge, dimmerRelayState);
}


// toggle the output relay, from phase--> 0-10 --> phase
void resetToggleRelayToPhase(int relaynumber)
{
  setdimmerRelayState(relaynumber, true);  // set to 0 - 10;
  delay(100);
  setdimmerRelayState(relaynumber, false); // back to phase mode
}

/*
    command 8 handler.
    set the dim mode ,   0 = 0-10 volt dc dim, / 1 = phase dim
*/
void hvDimMode (uint8_t data)
{
  Serial.print("NEW HV Dim Mode mask 0(phase dim) / 1(0-10volt)  = ");
  hvDimModeMask = data; //Fixme with Nick, logic inversion ... see ~  // nick removed inverstion, will correct lower.
  Serial.println(hvDimModeMask, HEX);
  relayUpdate();  // ngp added 8/3
}


// the dimmerrelays ,  are used / toggled for 2 reasons:
//if in 0 - 10 volt mode,  then when you go to 0 on pwm,  flip,
//if in phase dime mode,  and change dimmer edge, we need to toggle it (above), in pol change routine,
void relayUpdate(void)
{
  int i;
  int8_t mask = 0x01;
  Serial.println("Relay Update");
  for (i = 0; i < 8; i++) //Don't want to update all the relays at once!
  {
    // 0 == phase, 1 0-10, dim,
    bool isPhaseDim = ((hvDimModeMask & mask) == 0);
    bool pwmIsZero = (pwmData[i] == 0x0000);

    if (isPhaseDim || pwmIsZero) //if channel in (phase-dim-mode) or (PWM is zero) then we need to open the relay.
    {
      if ((dimmerRelayState & mask) != 0) //The relay was previously closed, we need to open it
      {
        setdimmerRelayState(i + 1, false);
        // dimmerRelayState&=~mask;
        // updateHVSR(dimmerEdge, dimmerRelayState);
        delay(100);
      }
    }
    else
    {
      if ((dimmerRelayState & mask) == 0) //The relay was previously open, we need to close it
      {
        setdimmerRelayState(i + 1, true);
        // dimmerRelayState|=mask;
        // updateHVSR(dimmerEdge, dimmerRelayState);
        delay(100);
      }
    }
    mask <<= 1;
  }

  printAllOutputConfigInfo();
}


void enQueue(uint8_t * data, int length) {

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
  if (rear >= QUEUE_SIZE - 1)
    rear = 0;


  //if(data[0] == 1)
  //    printQueue();
  // Serial.print(front);
}

void printQueue()
{
  for (int i = 0 ; i < 10; i++)
  {
    Serial.print(i);
    Serial.print(" --- ");
    for (int j = 0 ; j < 20 ; j++)
    {
      Serial.print(queue2[i][j]);
      Serial.print(" | ");
    }
    Serial.println(" ");

  }
}

void deQueuebuff(uint8_t * rxdata) {

  if (front == rear)
  {
    rxdata[0] = 0xff;
    return;
  }
  else {

    // Serial.print("DQ  ");
    // Serial.println(front);
    // Serial.println(rear);

    // Serial.print("got valid buffer data");
    // Serial.println(front);

    memcpy(rxdata, queue2[front], 200);

    memset(queue2[front], 0xFF, 200);

    front++;
    if (front >= QUEUE_SIZE - 1)
      front = 0;


    return;
  }
}

void MeasureWetDryContactInputs()
{
  int result = 0;
  for (int i = 3; i >= 0; i--)
  {
    result *= 2; // left shift 1
    if (digitalRead(dc[i]) == HIGH) result += 1;
  }
  wetdrycontactstatus[0] = 1; //length
  wetdrycontactstatus[1] = result;

//  Serial.println(wetdrycontactstatus[1],HEX);
}

void TxVersionInfo()
{
  uint8_t txbuff[4];
  Serial.println("got version request, sending now" );
  txbuff[0] = 3;
  txbuff[1] = LV_HV;
  txbuff[2] = RELEASENUMBERMAJOR;
  txbuff[3] = RELEASENUMBERMINOR;
  Wire.write(txbuff, 4);
}

void MeasureZero2TenInputs()
{
  int tempdata = 0;
  memset(zero2teninputstatus, 0, sizeof(zero2teninputstatus));
  zero2teninputstatus[0] = 8;

  tempdata = analogRead(analogIn[0]);
  zero2teninputstatus[1] = (uint8_t)(tempdata >> 8) & 0xFF;
  zero2teninputstatus[2] = (uint8_t)(tempdata & 0xFF);

  tempdata = analogRead(analogIn[1]);
  zero2teninputstatus[3] = (uint8_t)(tempdata >> 8) & 0xFF;
  zero2teninputstatus[4] = (uint8_t)tempdata & 0xFF;
  tempdata = analogRead(analogIn[2]);
  zero2teninputstatus[5] = (uint8_t)(tempdata >> 8) & 0xFF;
  zero2teninputstatus[6] = (uint8_t)(tempdata & 0xFF);

  tempdata = analogRead(analogIn[3]);
  zero2teninputstatus[7] = (uint8_t)(tempdata >> 8) & 0xFF;
  zero2teninputstatus[8] = (uint8_t)(tempdata & 0xFF);
}

void MeasurePWMCurrent()
{
  long data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int count = 0;

  elapsedMicros t = 0;
  while (t < 5000)
  {
    for (int i = 0; i < 8; i++)
    {
      data[i] += analogRead(iMeas[i]);
    }
    count++; //for div
  }

  memset(pwmcurrentstatus, 0, sizeof(pwmcurrentstatus));
  pwmcurrentstatus[0] = 16;
  for (int i = 0; i < 8; i++)
  {
    data[i] /= count;
    pwmcurrentstatus[(2 * i) + 1] = char(data[i] / 256);
    pwmcurrentstatus[(2 * i) + 2] = char(data[i] & 0xFF);
  }
}


/**
    set outputpol  (LV = outputpol  /HV = dimmer edge direction).
*/
void setOutputPolarity(uint8_t mode)
{
  if (LV_HV == 0)
  {
    Serial.print("Output Polarity\n");
    uint8_t opolData = mode;
    Serial.print("Output Polarity data= ");
    Serial.print(opolData);
    Serial.print("\n\n");
    shiftRegisterWriteByte(opolData);
  }
  else //High-voltage mode
  {
    Serial.print("-Dimmer edge Mask (1 fwd / 0 reverse) ");
    int oldDimmerEdge = dimmerEdge;
    dimmerEdge = mode;
    int newDimmerEdge = dimmerEdge;
    Serial.print(" | Existing = ");
    Serial.print(oldDimmerEdge, HEX);
    Serial.print("  | New = ");
    Serial.println(newDimmerEdge, HEX);
    int i;

    //
    if (oldDimmerEdge != newDimmerEdge)
    {
      // figure out which channels have changed phase direction,
      //we need to toggle the dimmer relay on that channel, (phase --> 0 -10 ..or vice versa..),  to update it,
      for (i = 0; i < 8; i++)
      {
        if ((newDimmerEdge & 0x00000001) != (oldDimmerEdge & 0x00000001))
        {
          Serial.print("Dimmer Edge Direction ");
          Serial.print(i);
          Serial.println(" changed!");
          resetToggleRelayToPhase(i + 1);
        }
        newDimmerEdge >>= 1;
        oldDimmerEdge >>= 1;
      }

    }
    printAllOutputConfigInfo();
  }
}

void setZero2TenVoltDrive(uint8_t * drivebuff)
{

  // Serial.println("0-10V Drive");
  // Serial.print("0-10V Drive data = ");
  int i;

  //
  //
  received = sizeof(drivebuff) / sizeof(drivebuff[0]);
  Serial.print("In setZero2TenVoltDrive function");
  Serial.println("Length of drivebuff = ");
  Serial.println(received);

  int numAin = received - 1; //numAin is the number of analog inputs to update.  We don't have to do all of them...

  Serial.print("received = ");
  Serial.println (received);
  if ((received - 1) > (sizeof(ainDrive) / 4)) //...but trying to update more than we have is bad.
  {
    Serial.println("We received too many bytes.  Smack Nick.");
    numAin = (sizeof(ainDrive) / 4);
  }

  for (i = 0; i < numAin; i++) //Received count is equal to the command plus all of its arguments.  "received-1" strips the command byte
  {
      Serial.print("i = ");
      Serial.print(i);
      Serial.print(" data = ");
    int driveValue = drivebuff[i + 1] * 256;
      Serial.println(driveValue);
    analogWrite(ainDrive[i], driveValue);
  }
}

void setPLCState(uint8_t stateval)
{
  //  Serial.print("PLC data= ");
  // Serial.println(stateval);
  for (int i = 0; i < 4; i++)
  {
    if (  ( (stateval >> i) & 0x01) > 0)
      digitalWrite(pSource[i], HIGH);
    else
      digitalWrite(pSource[i], LOW);
  }
}

void setPWMState(uint8_t * pwmvals)
{
  Serial.print("pwmvals: ");
  for (int i = 0; i < 8; i++)
  {
    int pwmValue = pwmvals[(2 * i) + 1] * 256 + pwmvals[(2 * i) + 2];
    int pct = (pwmValue * 100) / 65535;
    // Serial.print(pwmValue);
    Serial.print(pct);
    Serial.print(" | ");

    pwmData[i] = pwmValue; // store value for ref
    analogWrite(pwmPin[i], pwmValue);
  }
  Serial.println("");
  if (LV_HV == 1)
  {
    relayUpdate();
  }

}


void MeasureRMSCurrent(int channelNum)
{
  iWindowSum = 0;
  iWindowAvg = 0;
  iWindowPow2Sum = 0;
  iWindowPow2Mean = 0;
  long minval = 5000;
  long maxval = 0;

  memset(iWindow, 0, sizeof(iWindow) / sizeof(iWindow[0])); // not necessary but probably a good idea for debugging

  if (channelNum > 7) // valid channels are 0-7
  {
    Serial.print("Warning! Trying to measure current for invalid channel #: ");
    Serial.println(channelNum);
  }

  // Shift all data in the array left discard oldest data pt
  for ( int i = 0; i < len_iWindow - 1; i++ )
  {
    iWindow[i] = iWindow[i + 1];
    iWindowSum += iWindow[i];

    if( iWindow[i] > maxval)
       maxval = iWindow[i];

     if( iWindow[i] < minval)
       minval = iWindow[i];
  }
  // Meas new data pt, append to array
  iWindow[ len_iWindow - 1 ] = analogRead(iMeas[channelNum]);  // new meas data to last element in array
  iWindowSum += iWindow[ len_iWindow - 1 ];
  iWindowAvg = iWindowSum / len_iWindow;

  // Remove DC component and calculate square
  for ( int i = 0; i < len_iWindow; i++ )
  {
    iWindowPow2Sum += pow ( (iWindow[i] - iWindowAvg), 2);  //note double is same as float w/arduino
  }
  // Calculate RMS
  iWindowPow2Mean = iWindowPow2Sum / len_iWindow;
  iRMS = sqrt(iWindowPow2Mean);
  rmsCurrentArray[channelNum] = iRMS;

  // Serial.print("winavg: ");
 // Serial.println(iWindowAvg);

  // Serial.print("pow - winavg: ");
 // Serial.println(iWindowPow2Mean);


  Serial.print("min: ");
  Serial.println(minval);
  Serial.print("max: ");
  Serial.println(maxval);

}



/*
void MeasureRMSCurrent2(int channelNum)
{
  iWindowSum = 0;
  iWindowAvg = 0;
  iWindowPow2Sum = 0;
  iWindowPow2Mean = 0;

 // memset(iWindow, 0, sizeof(iWindow) / sizeof(iWindow[0])); // not necessary but probably a good idea for debugging

  // Shift all data in the array left discard oldest data pt
  for ( int i = 0; i < 200 - 1; i++ )
  {
    chan0rms_meas[i] = chan0rms_meas[i + 1];
    iWindowSum += chan0rms_meas[i];
  }

  chan0rms_meas[0] = analogRead(iMeas[channelNum]);  // new meas data to last element in array
   Serial.print("analog read: ");
    Serial.println(chan0rms_meas[0]);
  // Meas new data pt, append to array
  
  iWindowSum += chan0rms_meas[0];

  chan0avg = iWindowSum / 200;



  // Remove DC component and calculate square
  for ( int i = 0; i < len_iWindow; i++ )
  {
    iWindowPow2Sum += pow ( (chan0rms_meas[i] - chan0avg), 2);  //note double is same as float w/arduino
  }
  
  // Calculate RMS
  iWindowPow2Mean = iWindowPow2Sum / 200;
  iRMS = sqrt(iWindowPow2Mean);
  rmsCurrentArray[channelNum] = iRMS;
  
}
*/
void printdebug(char* str, int flag)
{
  if (flag)
  {
    Serial.print(str);
  }
}



// -------------------------------------------- Reboot logic -----------------------------


void softReboot(void)
{
  // ------------------------------------------
  // Pull RPi pin12/GPIO-18 High for 1 sec
  // RPi pin12 has internal pulldown enabled
  // Informs RPi to run sudo shutdown -r now
  // ------------------------------------------delay(1000);
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
{
  // ------------------------------------------------
  // Pull RPi Run low for 1 sec to reset/reboot
  // Teensy High connnects RPi Run low through Mosfet
  // ------------------------------------------------
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
{
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
  timeDeltaRx = millis() - timeStampMostRecentRx;
  Serial.print("Main Loop: timeDeltaRx = ");
  Serial.println(timeDeltaRx);

  if (timeDeltaRx > 1000 && softRebootTryNum < maxNumSoftRebootTries)
  {
    Serial.println("******************************");
    Serial.print("In Soft Reboot Loop ");
    Serial.print("Try # ");
    Serial.print(softRebootTryNum + 1);
    Serial.print("/");
    Serial.println(maxNumSoftRebootTries);
    Serial.println("******************************");

    softReboot();
    Serial.print("Waiting ");
    Serial.print(rebootWaitTimeMillis / 1000);
    Serial.println("sec for RPi to boot");
    delay(rebootWaitTimeMillis);
    Serial.print(rebootWaitTimeMillis / 1000);  // By then an rx event should have occured
    Serial.println(" seconds has elapsed");
    softRebootTryNum += 1;
  }

  if (timeDeltaRx > 1000 && softRebootTryNum == maxNumSoftRebootTries)
  {
    hardReboot();
    delay(2000);          // long enough for PI to reboot
    softRebootTryNum = 0; // does it matter that this is after delay(45s)?
    Serial.print("Waiting ");
    Serial.print(rebootWaitTimeMillis / 1000);
    Serial.println("sec for RPi to boot");
    delay(rebootWaitTimeMillis);
    Serial.print(rebootWaitTimeMillis / 1000);  // By then an rx event should have occured
    Serial.println(" seconds has elapsed");
  }
}


void setAllPWMtoLevel(uint16_t level)
{
  int i = 0;

  for (i = 0 ; i < 8; i++)
  {
    pwmData[i] = level; // store value for ref
    analogWrite(pwmPin[i], level);
  }

}


void setFailoverRelay(int relaynumber, bool state)
{
  if (state)
  {
    powerfailovermask |= (0x01 << (relaynumber - 1)); // set bit,
  }
  else
  {
    powerfailovermask &= ~(0x01 << (relaynumber - 1)); // clear bit,
  }
  updateHVSR(dimmerEdge, dimmerRelayState);
}


// Power failover pseudo code ****

void POEAndLinePowerFailoverHandler()
{
  boolean changed = false;

  byte linesense = shiftRegisterReadByte();

  boolean emergencypower_1_4  = (((linesense >> 0) & 0x01) >= 1); // bit 0
  boolean linepower_1_4 = (((linesense >> 1) & 0x01) >= 1);   // bit 1
  boolean emergencypower_5_8  = (((linesense >> 2) & 0x01) == 1);
  boolean linepower_5_8 = (((linesense >> 3) & 0x01) == 1);

  //  Serial.print("line 1-4 power: "  );
  //  Serial.println(linesense,HEX);
  // Serial.println(linepower_1_4);



  // check line power 1-4,
  if (!linepower_1_4) //if 0
  {
    if (linepower_1_4_failstart == 0x00) // if in init state.
    {
      linepower_1_4_failstart = millis();
      Serial.println("line 1-4 power lost detected");
    }
    else
    {
      long delta = millis() - linepower_1_4_failstart;
      if (delta >= 1000 && (powerfailovermask & 0x01) == 1) // wait 1 sec then start failover.
      {
        Serial.println("line 1-4 power failover starting now, switch to EMERGENCY power");
        powerfailovermask &= ~0x01; //|= 0x01;  // set 1-4  // swtich to emergency
        // set all outputs to 100%
        Serial.println("setting all outputs to 100% and ignoring requests. ");
        setAllPWMtoLevel(65535);
        failover_ignorerequests = 0x01;  // ignore all requests from outside,
        changed = true;
      }
    }
  }
  else   // line power is present,  make sure that everything is reset
  {
    linepower_1_4_failstart = 0x00; // rest time holder,
    if ((powerfailovermask & 0x01) == 0) // if the 1-4 side is in emergency mode, pull it out.
    {
      Serial.println("line 1-4 power going back to UTILITY now (reset)  ");
      powerfailovermask |= 0x01;  // clear the bit go back to utility,
      changed = true;
    }
  }




  // line 5- 8 ***********************************************************
  //
  if (!linepower_5_8) //if 0
  {
    if (linepower_5_8_failstart == 0x00) // if in init state.
    {
      linepower_5_8_failstart = millis();
      Serial.println("line 5-8 power lost detected");
    }
    else
    {
      long delta = millis() - linepower_5_8_failstart;
      if (delta >= 1000 && ((powerfailovermask >> 1) & 0x01) == 1) // wait 1 sec then start failover.
      {
        Serial.println("line 5-8 power failover starting now, switch to EMERGENCY power");
        powerfailovermask &= (~0x01 << 1); //|= 0x01;  // set 1-4  // swtich to emergency
        // set all outputs to 100%
        Serial.println("setting all outputs to 100% and ignoring requests. ");
        setAllPWMtoLevel(65535);
        failover_ignorerequests = 0x01;  // ignore all requests from outside,
        changed = true;
      }
    }
  }
  else   // line power is present,  make sure that everything is reset
  {
    linepower_5_8_failstart = 0x00; // rest time holder,
    if (((powerfailovermask >> 1) & 0x01) == 0) // if the 1-4 side is in emergency mode, pull it out.
    {
      Serial.println("line 5-8 power going back to UTILITY now (reset)  ");
      powerfailovermask |= (0x01 << 1); // &= ~0x01; // clear the bit go back to utility,
      changed = true;
    }
  }

  //********************************************************************************

  if (linepower_1_4 || linepower_5_8)
  {
    failover_ignorerequests = 0x00; // clear ignore flag,
  }

  // todo:  make sure that the failover mask is 0,  if both lines are ok ?,  and poe is ok,
  if (linepower_1_4 && linepower_5_8)
  {
    if (powerfailovermask == 0)
    {
      Serial.println("both lines are ok, and failover mask is not 0,  forceing back to 0");
      powerfailovermask = 0x03;
      changed = true;
    }
  }



  if (changed)
  {
    updateHVSR(dimmerEdge, dimmerRelayState);
  }

}


// debug print routines. **************************************************
void printOutputConfigInfo(int outputnumber)
{
  int mask = 0x01 << (outputnumber - 1);
  // 0 == phase, 1 0-10, dim,
  String dimmode = ((hvDimModeMask & mask) == 0) ? "Phase" : "0-10V";
  String phasedir = ((dimmerEdge & mask) == 0) ? "reverse" : "forward";
  String relaystate = ((dimmerRelayState & mask) == 0) ? "0" : "1";

  Serial.print(dimmode);
  Serial.print("      ");
  Serial.print(phasedir);
  Serial.print("        ");
  Serial.print(relaystate);
  Serial.print("            ");
  Serial.println(pwmData[outputnumber - 1]);
}


void printAllOutputConfigInfo()
{
  int i = 0;
  Serial.println("dim mode | phase dir | relay state  |  pwm val");
  for (i = 0; i < 8; i++) //Don't want to update all the relays at once!
  {
    printOutputConfigInfo(i + 1);
  }
}

// **************************************************************************


