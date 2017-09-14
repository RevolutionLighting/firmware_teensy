//-------------------------------------------------------------------------------------
// Revolution Lighting Project Firmware
//-------------------------------------------------------------------------------------
// Version 1.0
// Added HV phase support new way,  
// passes lv board test 9/14/17
//
// 

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
#define RELEASENUMBERMAJOR 1
#define RELEASENUMBERMINOR 0
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
void updateHVSR();
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


const int pwmPin_HVL[] = {2, 3, 4, 29, 6, 9, 10, 5};        // pwmPins swizzled pins to bank up all on one side,  1-4 are on ftm 1,2,3,   5-8  are on ftm 0

const int analogIn[] = {A22, A0, A1, A2};               //
const int n_analogIn = 4;                               //

const int plcOut[] = {27, 28, 11, 13};                 // PLC outputs 
const int n_plcOut =  4;                               //

const int ainDrive[] = {21, 22, 23, 30};                //
const int n_ainDrive = 4;                               //

const int dc[] = {12, 24, 25, 26};                      // Dry contact
const int n_dc = 4;                                     //

const int iMeas[] = {A14, A15, A16, A17, A18, A19, A20, A21}; // Current sense
const int n_iMeas = 8;

uint8_t ignoreLastRx = 0;                               // ***
volatile uint8_t received;                              // ***

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
//uint8_t powerfailovermask = 0x03;   // 0 is emergency ,  1 is utility line.

unsigned long linepower_1_4_failstart = 0x00;  // set when line power first fails. to detect 1 sec of fialure.
unsigned long linepower_5_8_failstart = 0x00;
byte failover_ignorerequests = 0x00;


long chan0rms_meas[200]; // chan 0 holds analog read values (raw)
long chan0avg = 0x00;  // for debug, to fix...
//-------------------------------------------------------------------------------------

int PWM_FREQ = 200;

byte togglebit = 0x00;
unsigned long pulsestarttime = 0;
byte resetpulse = 0x01;
byte wdcontact_state_changed_mask = 0;  //flag when wd contact is read out from rpi


// Phase Dimming for HV board .vars 
long lastsr_d_in_value = 0;  // last value read in from the shift reg  data val.  , for calc zero cross time, 
unsigned int sync_pulse_count = 0;  // the number of pulses read in 
unsigned long accum_pulse_durations = 0;  // total pulse lengths (accumulation)... is divided to get avg,...over window
//byte active_bank = 0; // 0 = (bank 1-4),  1 = bank 5-8 // this is the bank currently being measured. 
unsigned long falling_edge_sync = 0;  // the last time we got a falling edge on sync, used to measure pulse widths. for zero cross measurments. 

unsigned int NUM_PULSES_TO_AVG = 32; //64;
//PhaseDimmer* phaseDimmers = new PhaseDimmer[8];

// new power failure/ shift reg stuff. 
//NICK NOTES: 9/7/17
// HV shift reg changes for rev c, 
// 16 bits total for write side:
// bits 0 - 7 ,  0-10v mode 
// bit 8  psource 0  // power fail over,  X
// bit 9 psource 1   // pfo ...X

// bit 10 ac sens sel0  // for reading src 0 
// bit 11    sel 1
// bit 12    sel 2
// bit 13    sel 3
uint8_t hv_dimmermode = 0; //  used for tracking mode set by user 0-10 or phase dim. 0 is phase  / 1 is 0-10
uint8_t hv_dimmerrelaystate = 0;  //var used for holding dimmer relay state. open / closed (1) 
boolean hv_psource0 = true;  //var used for holding state of power source (line/emergency ) for outs 1-4  // assume we have power --- true is Util,  false is aux. 
boolean hv_psource1 = true;  //var used for holding state of power source (line/emergency ) for outs 5-8
uint8_t hv_ac_sens_src = 1;  //var used for switching the ac source for reading zero cross/level.  one of 4 sources  (val = 0 - 3) 
int dimmerEdge = 0x00;   // var holder, for holding phase direction (forward or reverse) for hv, ,  set by setting outputpolarity command from host,  (shared with LV output pol). 

// vars used for holding the last valid time power was detected on given src number, (used for power failover logic). 
// there are in mills
unsigned long last_valid_power_src0 = 0;
unsigned long last_valid_power_src1 = 0;
unsigned long last_valid_power_src2 = 0;
unsigned long last_valid_power_src3 = 0;

unsigned long last_hv_ac_sens_src_switch_mills = 0;


// PWM Current measurment (LV) new non blocking way 9/14/17
int pwmcurrentsamplecount = 0;   // number of samples taken per channel
long pwm_sample_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// -------------------------------------------------------------------------------------------

void setup()
{

  if (LV_HV == 1)
    PWM_FREQ = 120;

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
  // PWM outputs (LV 24 pwm,  on hv  its either 0-10 or phase) depending on mode. 
  for (i = 0; i < 8; i++)
  {
    if (LV_HV == 0)
    {
      pinMode(pwmPin[i], OUTPUT);
      analogWriteFrequency(pwmPin[i], PWM_FREQ);  //PWM freq
      analogWrite(pwmPin[i] , 0);
    }
    else
    {
       pinMode(pwmPin_HVL[i], OUTPUT);
       analogWrite(pwmPin_HVL[i] , 32767);
       analogWriteFrequency(pwmPin_HVL[i], 119.97);  //PWM freq
    }
  }

  // PLC outputs 
   for (i = 0; i < 4; i++) // length of total array / length of element (byte) = # elements
  {
      pinMode(plcOut[i], OUTPUT);
      digitalWrite(plcOut[i], LOW);
  }

  // Dry Contacts 
  pinMode(dc[0], INPUT);
  pinMode(dc[1], INPUT);
  pinMode(dc[2], INPUT);
  pinMode(dc[3], INPUT);


 // pwm current measurment pins
  for (i = 0; i < (sizeof(iMeas) / 4); i++)
  {
    pinMode(iMeas[i], INPUT);
  }

  // 0-10 volt current drive(active devices) --- this is not enabled offically becuase of ftm criss - cross. 
  for (i = 0; i < (sizeof(ainDrive) / 4); i++)
  {
    pinMode(ainDrive[i], OUTPUT);
  //  analogWriteFrequency(ainDrive[i], PWM_FREQ); //  8/22 changed from 10K, to 200 Hz, to fix pwm channels. 10000);     // 10kHz PWM freq
  }

  // 0-10 volt inputs
  for (i = 0; i < (sizeof(analogIn) / 4); i++)        
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

  last_valid_power_src0 = millis();
  last_valid_power_src1 = millis();
  last_valid_power_src2 = millis();
  last_valid_power_src3 = millis();
  setHV_AC_SENS_SRC(1);

  
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
  {
   // Serial.println("  pwm current meas start");
    MeasurePWMCurrent();
   // Serial.println(" pwm current meas stop");
  }


   // ********************************************************************************************************
   // ****************************************************************** PHASE DIM FOR HV ********************
   // ********************************************************************************************************
  if (LV_HV == 1)
  {
   
   // this is proto for pwm way
    // simple check for zero cross, and after X sec, (last sync,  reset the timer(ftm3) to 0,  to sync it, to zero cross. 
    
    unsigned long current_timemicros = micros();
    unsigned long current_timemillis= millis();
    // read sr data in,  (will be shift reg read )
    long bitVal = digitalReadFast(SR_DAT_IN);

     if(current_timemillis - last_hv_ac_sens_src_switch_mills >= 50)  // 50ms delay between switchs. 
     {
         // todo once shift regs are in. 
        if (lastsr_d_in_value == 0 && bitVal == 1) //rising edge(forward edge)
        {
          lastsr_d_in_value = 1;  // just set state change,  do nothing else. 
         //  Serial.println(" low to high");
        }
        else if (lastsr_d_in_value == 1 && bitVal == 0) // falling edge  (reverse) 
        {
          lastsr_d_in_value = 0;
          if(falling_edge_sync == 0)
          {
             falling_edge_sync = current_timemicros;  // update edge time only
          }
          else
          {
            unsigned long zero_cross_pulse_duration_reverse = current_timemicros - falling_edge_sync;  // calc zero cross pulse duration, (should be around 16660+ us, ) 
            falling_edge_sync = current_timemicros;  // update edge time
            accum_pulse_durations += zero_cross_pulse_duration_reverse;
            sync_pulse_count++;
          }

          int pulsecnt_threshold = 8;
          if(hv_ac_sens_src == 0 || hv_ac_sens_src == 2)
          {
              pulsecnt_threshold = 4;
          }
          
          if(sync_pulse_count >= pulsecnt_threshold)  // *********************** TIME TO Take measurment,, , ***********************
          {
            // Serial.println(accum_pulse_durations);
             falling_edge_sync = 0; //reset of vars. 
             sync_pulse_count = 0;
            unsigned long avg_dur_us = accum_pulse_durations/pulsecnt_threshold; 
            accum_pulse_durations = 0; 
            float avg_freq = (float)1000000/(float)avg_dur_us;
            float basefreq = 2.00*avg_freq;
           // Serial.print(getAC_SensSrcName(hv_ac_sens_src)); 
           
            if(avg_freq > 58 && avg_freq < 62)
            {
                //  Serial.print(" (VALID) = "); 
                //  Serial.print(hv_ac_sens_src);
                 // Serial.print("  :  " );
                //  Serial.println(avg_freq);
                 if(hv_ac_sens_src == 1 || hv_ac_sens_src == 3)  //only sync on util lines.  not aux. 
                 {                 
                    //basefreq = 120.06;
                    //Serial.print("Updated pwm for src " );
                    // Serial.print(hv_ac_sens_src);
                    // Serial.print("  :  " );
                   // Serial.println(basefreq);
                    setPWMBaseFrequency(basefreq,hv_ac_sens_src);  // NOTE:   this need to be enabled ,,,, based on wire change becuase of banking of pwms...and (REWIRE)....
                 }
                   
                 switch(hv_ac_sens_src)
                 {
                     case 0:
                        last_valid_power_src0 = current_timemillis;
                      break;
                     case 1:
                         last_valid_power_src1 = current_timemillis;
                       break;
                       case 2:
                          last_valid_power_src2 = current_timemillis;
                       break;
                       case 3:
                       last_valid_power_src3 = current_timemillis;
                       break;
                       default:
                       break;
                 } 
            }
            else
            {
                // Serial.print(" (INVALID VALUE (Ignored) = "); 
                // Serial.println(avg_freq);
            }
    
            //switch bank we are looking at. 
           setNextACSensSrc();
           
          }
          // digitalWrite(pwmPin[0], LOW);  //for debug
         // Serial.println("high to low");
        }
        else if(current_timemillis - last_hv_ac_sens_src_switch_mills > 1000)
        {
            falling_edge_sync = 0; //reset of vars. 
            sync_pulse_count = 0;
            accum_pulse_durations = 0; 
           // Serial.print(getAC_SensSrcName(hv_ac_sens_src)); 
           // Serial.println(" (TIMEOUT: !) = assumed 0 "); 
           setNextACSensSrc();
        }


     }

   
    
    PowerFailOverTestAndHandle();
  }  // end if HV 

  

  // ********************************************************************************************************
  // ********************************************************************************************************
  // *********************************** END phase mode dev ******************************
  // ********************************************************************************************************

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
  //Serial.println(micros());
  
  loopcount++;                 //10000 == 25ms,
  if (loopcount >= 1000)     // 40000 == 100ms,
  {
    // Serial.println(micros());
    loopcount = 0; 
    MeasureZero2TenInputs();  // this is non blocking. ...
    MeasureWetDryContactInputs();  // this is non  blocking, 
    //Serial.println("loop rollover");
  }
  
} // end main loop()

void setNextACSensSrc()
{
   uint8_t nextsrc = 1;  //swizzle,  0 , 2, 1, 3,, ..0 
   
  /*  if(hv_ac_sens_src == 1)
    {
        nextsrc = 3;
    }
    else
        nextsrc = 1;
     */   
     
     
      
      switch (hv_ac_sens_src)
      {
        case 0:
           nextsrc = 2;
           break;
        case 1:
            nextsrc = 3;
           break;
        case 2:
           nextsrc = 1;
           break;
        case 3:
           nextsrc = 0;
           break;
         default:
         break;
      }  
      setHV_AC_SENS_SRC(nextsrc);
}

char* getAC_SensSrcName(uint8_t srcnum)
{
    switch(srcnum)
    {
      case 0:
      return "AUX  (1-4)";
      case 1:
      return "LINE (1-4)";
      case 2:
      return "AUX  (5-8)";
      case 3:
      return "LINE (5-8)";
      default:
      return "UNKNOWN";
    }
}

void setPWMBaseFrequency(float freq, int bank) // boolean forward)
{
  if(bank == 0 || bank == 1)
  {
     // Serial.print("bank 0/1 freq: ");
     //  Serial.println(freq);
     analogWriteFrequency(pwmPin_HVL[0], freq);  // these are on ftm 1,2,3
     analogWriteFrequency(pwmPin_HVL[1], freq);   
     analogWriteFrequency(pwmPin_HVL[2], freq);   
     analogWriteFrequency(pwmPin_HVL[3], freq);   
  }
  else if (bank == 2 || bank == 3)              // NOTE ,  these are all on FTM 0
  {
    //  Serial.print("bank 2/3 freq: ");
    //   Serial.println(freq);
     analogWriteFrequency(pwmPin_HVL[4], freq);  
  //   analogWriteFrequency(pwmPin_HVL[5], freq);   
   //  analogWriteFrequency(pwmPin_HVL[6], freq);   
   //  analogWriteFrequency(pwmPin_HVL[7], freq);   
  }
}


void setPWMState(uint8_t * pwmvals)
{
  Serial.print("pwmvals: ");

  for (int i = 0; i < 8; i++)
  {
    int pwmValue = pwmvals[(2 * i) + 1] * 256 + pwmvals[(2 * i) + 2];  // extract value from array that is sent over. from host. 
    int pct = (pwmValue * 100) / 65535;  // convert it to a pct value. 

    int dutyValueReverse=int(float(pct*0.01)*65535);    // rising edge. (forward)
    int dutyValueForward=int((1-float(pct*0.01))*65535);  // falling edge, (reverse) 

    //Serial.print(dutyValueForward);
    //Serial.print(" | ");
    //Serial.print(dutyValueReverse);
    //Serial.print(" | ");
   // Serial.print(pwmValue);
    //Serial.print(" | ");
    Serial.print(pct);
    Serial.print(" | ");

  
    if (LV_HV == 0)   // 8/23/17,   only for lv now.
    {
      analogWrite(pwmPin[i], pwmValue);
      pwmData[i] = pwmValue; // store value for ref
    }
    else  // HV
    {
        int val = pwmValue; 
        if(((dimmerEdge >> i) & 0x01) > 0)  // if forward
        {
           analogWrite(pwmPin_HVL[i],dutyValueForward);
           pwmData[i] = dutyValueForward;
           
        }
        else
        {
            analogWrite(pwmPin_HVL[i],dutyValueReverse);
            pwmData[i] = dutyValueReverse;
        }
        // note need to add hvdimm mode check here too, to make sure not 0 -10 volt.    
        
    }
  }
  Serial.println("");

  if (LV_HV == 1)
  {
   // Serial.println("this is hv ,  no pwm adjust, using new alg" );
    relayUpdate();
  }

}

// *****************************************************************************************************************************
// *****************************************************************************************************************************
//                                         I2C recieve command to set something (write),
//
// *****************************************************************************************************************************
// *****************************************************************************************************************************
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


// *****************************************************************************************************************************
// *****************************************************************************************************************************
//                                         I2C recieve command to GET something (READ),
//
// *****************************************************************************************************************************
// *****************************************************************************************************************************
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
    wdcontact_state_changed_mask = 0;
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



void setdimmerRelayState(int relaynumber, bool state)  // reminder ..state 1 (phase)  /0 = 0 -10 volt.
{
  if (state)  // **************************************************************   NGP 9/8/17 inverted per ward at 2pm ,
  {                
   
    hv_dimmerrelaystate |= (0x01 << (relaynumber - 1)); // set bit,
  }
  else
  {
    hv_dimmerrelaystate &= ~(0x01 << (relaynumber - 1)); // clear bit,
    
  }
  Serial.print("hv_dimmerRelayState: ");
  Serial.println(hv_dimmerrelaystate, HEX);
  updateHVSR();
}


// toggle the output relay, from phase--> 0-10 --> phase
void resetToggleRelayToPhase(int relaynumber)
{
  setdimmerRelayState(relaynumber, false);  // set to 0 - 10;
  delay(100);
  setdimmerRelayState(relaynumber, true); // back to phase mode
}

/*
    command 8 handler.
    set the dim mode ,   0 = 0-10 volt dc dim, / 1 = phase dim
*/
void hvDimMode (uint8_t data)
{
  Serial.print("NEW HV Dim Mode mask 0(phase dim) / 1(0-10volt)  = ");
  hv_dimmermode = data;
  Serial.println(hv_dimmermode, HEX);
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
    bool isPhaseDim = ((hv_dimmermode & mask) == 0);
    bool pwmIsZero = (pwmData[i] == 0x0000);

    if (isPhaseDim || pwmIsZero) //if channel in (phase-dim-mode) or (PWM is zero) then we need to open the relay.
    {
      if ((hv_dimmerrelaystate & mask) == 0) //The relay was previously closed, we need to open it
      {
        setdimmerRelayState(i + 1, true);   // set to phase dim mode. 
        delay(100);
      }
    }
    else
    {
      if ((hv_dimmerrelaystate & mask) != 0) //The relay was previously open, we need to close it
      {
        setdimmerRelayState(i + 1, false);  // set to 0 - 10 Volt mode
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

    for (int i = 0; i < 8; i++)
    {
      pwm_sample_data[i] += analogRead(iMeas[i]);
    }
    pwmcurrentsamplecount++; //for div
    
  if(pwmcurrentsamplecount > 2000)
  {
    //  Serial.println("PWM measurment done");
    memset(pwmcurrentstatus, 0, sizeof(pwmcurrentstatus));
    pwmcurrentstatus[0] = 16;
    for (int i = 0; i < 8; i++)
    {
      pwm_sample_data[i] /= pwmcurrentsamplecount;
      pwmcurrentstatus[(2 * i) + 1] = char(pwm_sample_data[i] / 256);
      pwmcurrentstatus[(2 * i) + 2] = char(pwm_sample_data[i] & 0xFF);
    }

     pwmcurrentsamplecount = 0;
     memset(pwm_sample_data, 0, sizeof(pwm_sample_data));
    
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
   // if (oldDimmerEdge != newDimmerEdge)  // removed 
   // {
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


         // 9/7/17  for new 
         int edgeconfig = 0x002C;  //default is reverse. 
         if((dimmerEdge >> i) & 0x01)  
         {
            edgeconfig = 0x0028;   // forward. 
         }

               // NEW PORT MAP...   = {2, 3, 4, 29, 6, 9, 10, 5};
           switch(i)
           {
              case 0:       //pwm0 pin2, ftm3, ftmch 0
                 FTM3_C0SC = edgeconfig;
                 // Serial.print("FTM Reg updated to: ");
                 // Serial.println(edgeconfig, HEX);
                 break;
               case 1:    //pwm1 pin3, ftm1, ftmch 0
                 FTM1_C0SC = edgeconfig;
                 break;
              case 2:    //pwm2 pin4, ftm1, ftmch 1
                 FTM1_C1SC = edgeconfig;
                 break;
              case 3:   // pwm3 pin29, ftm2, ftmch 0
                 FTM2_C0SC = edgeconfig;
                 break;
                 // **********************************************
              case 4:  // pwm4 pin6, ftm0, ftmch 4
                FTM0_C4SC = edgeconfig;
                 break;
              case 5:  // pwm5 pin9, ftm0, ftmch 2
                FTM0_C2SC = edgeconfig; 
                 break;
              case 6:  // pwm6 pin10, ftm0, ftmch 3
                FTM0_C3SC = edgeconfig;  
                 break;
              case 7:  // pwm7 pin5, ftm0, ftmch 7
                FTM0_C7SC = edgeconfig; 
                 break;
                 default:
                 break;
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
  // Serial.print("PLC data= ");
  // Serial.println(stateval, HEX);
  for (int i = 0; i < 4; i++)
  {
    if (  ( (stateval >> i) & 0x01) > 0)
      digitalWrite(plcOut[i], HIGH);
    else
      digitalWrite(plcOut[i], LOW);
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

    if ( iWindow[i] > maxval)
      maxval = iWindow[i];

    if ( iWindow[i] < minval)
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


void setPWMBanktoLevel(uint16_t level, int bank)
{
  if(bank == 0 || bank == 1)
  {
     analogWrite(pwmPin_HVL[0], level);
     analogWrite(pwmPin_HVL[1], level);
     analogWrite(pwmPin_HVL[2], level);
     analogWrite(pwmPin_HVL[3], level);
  }
  else if(bank == 2 || bank == 3)
  {
     analogWrite(pwmPin_HVL[4], level);
     analogWrite(pwmPin_HVL[5], level);
     analogWrite(pwmPin_HVL[6], level);
     analogWrite(pwmPin_HVL[7], level);
  }
 

}

/*    for testing pupose only 
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
**************************************/

// Power failover pseudo code ****

void PowerFailOverTestAndHandle()
{
  boolean changed = false;
  unsigned long current_timemillis = millis();
  unsigned long FAILURE_TIME_MS = 2500;  // time in MS,  till power is said to be 0,  then wait anothe 1 sec... and start failover. 
   
  boolean emergencypower_1_4  = ((current_timemillis - last_valid_power_src0) < FAILURE_TIME_MS);
  boolean linepower_1_4 = ((current_timemillis - last_valid_power_src1) < FAILURE_TIME_MS);
  boolean emergencypower_5_8  = ((current_timemillis - last_valid_power_src2) < FAILURE_TIME_MS);
  boolean linepower_5_8 = ((current_timemillis - last_valid_power_src3) < FAILURE_TIME_MS);

   // Serial.print("line 1-4 power: "  );
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
      if (delta >= 1000 && hv_psource0) // wait 1 sec then start failover.
      {
        Serial.println("line 1-4 power failover starting now, switch to EMERGENCY power");
        hv_psource0 = false;
        // set all outputs to 100%
        Serial.println("setting outputs 1-4  to 100% and ignoring requests. ");
        setPWMBanktoLevel(65535,0);
        failover_ignorerequests = 0x01;  // ignore all requests from outside,
        changed = true;
      }
    }
  }
  else   // line power is present,  make sure that everything is reset
  {
    linepower_1_4_failstart = 0x00; // rest time holder,
    if (!hv_psource0) // if the 1-4 side is in emergency mode, pull it out.
    {
      Serial.println("line 1-4 power going back to UTILITY now (reset)  ");
      hv_psource0 = true;
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
      if (delta >= 1000 && hv_psource1) // wait 1 sec then start failover.
      {
        Serial.println("line 5-8 power failover starting now, switch to EMERGENCY power");
        hv_psource1 = false;
        // set all outputs to 100%
        Serial.println("setting outputs (5-8) to 100% and ignoring requests. ");
        setPWMBanktoLevel(65535,2);
        failover_ignorerequests = 0x01;  // ignore all requests from outside,
        changed = true;
      }
    }
  }
  else   // line power is present,  make sure that everything is reset
  {
    linepower_5_8_failstart = 0x00; // rest time holder,
    if (!hv_psource1) // if the 1-4 side is in emergency mode, pull it out.
    {
      Serial.println("line 5-8 power going back to UTILITY now (reset)  ");
       hv_psource1 = true;
     // powerfailovermask |= (0x01 << 1); // &= ~0x01; // clear the bit go back to utility,
      changed = true;
    }
  }

  //********************************************************************************
   
  if (linepower_1_4 || linepower_5_8)
  {
    failover_ignorerequests = 0x00; // clear ignore flag,
  }

  // if both lines are OK,  and both sources are set to AUX,  set both to line,  this is just a catch all,  
  if (linepower_1_4 && linepower_5_8)
  {
    if (!hv_psource0 && !hv_psource1)  
    {
      Serial.println("both lines are ok, and failover mask is not 0,  forceing back to 0");
      hv_psource0 = true;
      hv_psource0 = true;
      changed = true;
    }
  }  



  if (changed)
  {
    updateHVSR();
  }


    
}


// debug print routines. **************************************************
void printOutputConfigInfo(int outputnumber)
{
  int mask = 0x01 << (outputnumber - 1);
  // 0 == phase, 1 0-10, dim,
  String dimmode = ((hv_dimmermode & mask) == 0) ? "Phase" : "0-10V";
  String phasedir = ((dimmerEdge & mask) == 0) ? "reverse" : "forward";
  String relaystate = ((hv_dimmerrelaystate & mask) == 0) ? "0" : "1";

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


   printHVSRInfo();
}

void printHVSRInfo()
{
  Serial.println("psource0 | psource1  |  ac sens source mask");
 // Serial.print(hv_dimmerrelaystate, HEX);
 // Serial.print("  |  ");
  Serial.print(hv_psource0);
  Serial.print("   |     " );
  Serial.print(hv_psource1);
  Serial.print("   |      " );
  Serial.println(hv_ac_sens_src);

  
}

// **************************************************************************
// test code for flipping ac sens source 
  //if(loopcount == 2)
  //{
    //Serial.println("on");
     //  hv_psource0 = true;
    
    // setdimmerRelayState(1,true);
     
    //  setHV_AC_SENS_SRC(0);
    //updateHVSR();
 // }
 // else if(loopcount == 1000000)
 // {
     //Serial.println("off");
     //   hv_psource0 = false;
      //   updateHVSR();
    
   // setdimmerRelayState(1,false);
    //  setHV_AC_SENS_SRC(1);
 // }
  
  // 
  // ************************* TEST CODE FOR FLIPPING RELAYS ****

  // loop test for normal output relays , channels 1- 8
 // if(loopcount == 200)
  //  setdimmerRelayState(1,true);
  // else if(loopcount == 700)
  //   setdimmerRelayState(1,false);


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
