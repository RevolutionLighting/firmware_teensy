//-------------------------------------------------------------------------------------
// Revolution Lighting Project Firmware
//-------------------------------------------------------------------------------------


// version 1.3 
// lv - speed up of current measurments  1/12/18

//version 1.2
// rev d hvh bringup

// version 1.1
// fixed issue with version number not getting accross.
// unified the pwm current meas into one func for both hv and lv.
// passed LV test 9/20/17

// Version 1.0
// Added HV phase support new way,
// passes lv board test 9/14/17

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
uint8_t RELEASENUMBERMAJOR = 1;
uint8_t RELEASENUMBERMINOR = 3;
#define LV_HV 1 //LV=0, HV=1
#include <arduino.h>
#include <i2c_t3.h>
boolean MEASURE_LOOPTIME = false;

boolean HV_POWER_FAILOVER = true;

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
void softRebootRPI(void);
void hardRebootRPI(void);
void rebootResolveI2C(void);
void enQueue(uint8_t * data, int length);
void printQueue(void);
void deQueuebuff(uint8_t * rxdata);
void TxVersionInfo(void);  // sends out version info, util func.
void MeasurePWMOutputCurrent(void);
// methods for measurements
void MeasureWetDryContactInputs(void);
void MeasureZero2TenInputs(void);
//void MeasureLV_PWMCurrent(void);   // take pwm current measurment


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

const unsigned long REBOOT_WAITTIME_MS = 45000;             // 45 seconds time to wait(delay) if soft or hard reboot is issued
unsigned long timeStampMostRecentRx;                    // loop(): Zero indicates no Rx event received yet

unsigned long timeLastRebootRequest = 0;                    // time of the last soft/hard reset request to pi.
boolean waitingForRPIBoot = false;
int softRebootTryNum = 0;                               // count of number of soft resets,
int maxNumSoftRebootTries = 3;                          // max # soft reboot tries before hard rebooting

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


// temp code for testing resets 9/14/17
long temp_softreset_countdown = 40000;


float last_5_8_freq = 120.00;  //for debug.

int current0_10_meas_index = 0;  // 0 -3  used for the 0-10 volt measuring ,  inc'd per X number of cycles,

// for debug,  measuring cycle (loop) times.
long test_last_loop_us = 0;
float maxcycletime = 0;
float avgcycletime = 0.0;
float mincycletime = 0xFFFFF;
// ***************************************


// NGP --- new hv current meas stuff 9/19/17

long rms_sample_start_time = 0;

uint8_t rms_channel_meas = 0;  // which channel are we measuing.
unsigned long rmsAccumHolder = 0;  // holds current accum value.
unsigned long lastsampletime = 0;   // time stamp of when last sampel was read in and added to accum
unsigned long num_samples_taken = 0;  // number of samples taken ,(divistor to calc avg. )
unsigned long sampleHolder[1000];  // holds samples during measurment cycle.
unsigned long samplemin = 0xFFFF;  //debug only
unsigned long samplemax = 0;  //debug only

uint8_t hw_info_buff[4]; // hw info buff/  board type/version


// 9/29/16
// new output relay shutoffs
uint8_t bias_enable = 0xFF;

unsigned long bitwigglecount = 0;


unsigned long delaypwmupdate_ms = 0;
boolean togglebias = false;


enum ZERO_CROSS_MEAS_STATE {
  MEASURING,
  SWITCH_SRC,
  WAITING_TO_SETTLE
};

ZERO_CROSS_MEAS_STATE zero_cross_state = MEASURING;

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
  digitalWrite(PI_RUN_RESET, LOW);


  pinMode(PI_GPIO18_PIN12, OUTPUT); // Soft reboot
  digitalWrite(PI_GPIO18_PIN12, LOW);  // default to low
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

  Serial.print("Board Type: ");
  Serial.println(LV_HV);
  //delay must take place before interupt handlers are enabled
  delay(1000);

  // Register wire/I2c slave received and request handlers
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // Setup for Slave mode, address 0x66, pins 18/19, external pullups, 400kHz
  Wire.begin(I2C_SLAVE, 0x66, I2C_PINS_7_8, I2C_PULLUP_EXT, 400000);
  delay(1000);  // wait 1 sec for i2c to start before main loop

  // Write shiftreg
  shiftRegisterWriteWord16(0x0001);

  Serial.println("loop start-- 9 ");

  last_valid_power_src0 = millis();
  last_valid_power_src1 = millis();
  last_valid_power_src2 = millis();
  last_valid_power_src3 = millis();
  setHV_AC_SENS_SRC(1);

  // ngp 9/14/17, assume we have rx atleats one packet...for rpi reboot logic.
  timeStampMostRecentRx = 0; //millis()-10000;

  // used for measuing loop time,  can be removed eventually.
  test_last_loop_us = micros();

  // setup the zero cross isr
  if (LV_HV == 1)
  {
    attachInterrupt(digitalPinToInterrupt(SR_DAT_IN), ZeroCrossISR, FALLING);
    setNextACSensSrc();
  }

  // HW INFO BUFFER SETUP
  hw_info_buff[0] = 3;
  hw_info_buff[1] = LV_HV;
  hw_info_buff[2] = RELEASENUMBERMAJOR;
  hw_info_buff[3] = RELEASENUMBERMINOR;


}	// End setup()

volatile long zc_isr_count = 0;
volatile long isr_start_meas = micros();
volatile float lastisr_meas = 0.0;

//****************************************************************************************************
// ********************************************* Zero Cross ISR **************************************

// NOTES:  10/23/17,
//zero cross count states:
// 0 = measuring window state, if here, isr will taking in counts  next state -1,
// -1 == tell system to switch the source number,  then go into -2,
// -2  = wait sate, used to wait between measurment windows. look in loop for duration (approx 3 ms) next state is 0,


// as of board rev D.  here is the pin/port mapping
// Sens0 - Aux0     
// Sens1 - Line 0              
// Sens2 - Aux1
// Sens3 - Line 1

//****************************************************************************************************
void ZeroCrossISR() {


  int PULSE_CNT_LIMIT = 8;
  if (hv_ac_sens_src == 0 || hv_ac_sens_src == 2)
    PULSE_CNT_LIMIT = 4;// do 6 pulses just for aux inputs,  to save time,


// if in measuing state. 
  if(zero_cross_state == MEASURING) // if (zc_isr_count >= 0)
  {
    if (zc_isr_count == 0)
      isr_start_meas = micros();

    zc_isr_count++;

    if (zc_isr_count >= PULSE_CNT_LIMIT)
    {
      long delta = micros() - isr_start_meas;
      lastisr_meas = (float)delta / (float)(zc_isr_count - 1);

      float avg_freq = (float)1000000 / (float)lastisr_meas;
      float basefreq = 2.00 * avg_freq;

      // isr_start_meas = micros();
      // if (avg_freq > 58 && avg_freq < 62)  10/23 changed thresh to 120,,
      if (basefreq > 118 && basefreq < 122)   // <--------------note only update if in valid range.
      {

        
        Serial.print(getAC_SensSrcName(hv_ac_sens_src));
        Serial.print(" : ");
        Serial.println(basefreq);
        
        
        
        if (hv_ac_sens_src == 1 || hv_ac_sens_src == 3)
          setPWMBaseFrequency(basefreq, hv_ac_sens_src); 
        // update presense times

        switch (hv_ac_sens_src)
        {
          case 0:  // aux 0
            last_valid_power_src0 = millis();
            break;
          case 1:   // line 0
            last_valid_power_src1 = millis();
            break;
          case 2:   // aux 1
            last_valid_power_src2 = millis();
            break;
          case 3:   // line 1
            last_valid_power_src3 = millis();
            break;
          default:
            break;
        }
      }
       zero_cross_state = SWITCH_SRC;
       //zc_isr_count = -1;  //
    }
  }

}
void loop()
{
  //-------------------------------------------------------------------------------------
  // Main Loop
  //-------------------------------------------------------------------------------------
  //-------------------------------------------------------------------------------------
  uint8_t rxdata[200];      // used for pulling in latest from queue.
  memset(rxdata, 0, sizeof(rxdata));  // /4

  deQueuebuff(rxdata);  // dequeue from set que,  is anything incomming,

  // if valid data from queue,  process it,
  // *********************************************************
  if (rxdata[0] != 0xff)
  {
    //Serial.print("valid data received; command byte = ");
    //Serial.println(rxdata[0]);
    
    //if (failover_ignorerequests == 0x00)
    //{
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
   // }
   // else
   // {
   //   Serial.println(" command ignored,  in failover mode");
   // }
  }
  else
  {
    // Serial.println("nothing in set queue");
    // do nothing ,  nothing to process from i2c
  }

  // measurement routines
  // PWM output current(both HV / LV) ,
  MeasurePWMOutputCurrent(); // unified non blocking,  per channel... periodic.

  // ********************************************************************************************************
  // ****************************************************************** HV********************
  // ********************************************************************************************************
  if (LV_HV == 1)
  {
    // Start of zero cross state machine advance /test code.
    // Zero cross is measured using isr counts,  in isr routine above, but state is tested here,  and advanced

    unsigned long current_timemicros = micros();
    unsigned long current_timemillis = millis();

    //if (zc_isr_count == -1) // switch source
    if(zero_cross_state == SWITCH_SRC)
    {
      setNextACSensSrc();

       zero_cross_state = WAITING_TO_SETTLE;
     // zc_isr_count = -2;
      
    }
    else if (zero_cross_state == WAITING_TO_SETTLE)  // else if (zc_isr_count == -2)
    {
      if (current_timemillis - last_hv_ac_sens_src_switch_mills >= 10)  // if over the settle period. 
      {
        zero_cross_state = MEASURING;
        zc_isr_count = 0;  // reset the isr count
      }

    }

    // if last time has been more than 1000 ms,  then timeout.  and auto switch(in caes pwr src is not present.
    if (current_timemillis - last_hv_ac_sens_src_switch_mills > 1000)
    {
       zero_cross_state = WAITING_TO_SETTLE;
     // zc_isr_count = -2;
      Serial.print(getAC_SensSrcName(hv_ac_sens_src));
      Serial.println(" (TIMEOUT: !) = assumed 0 ");
      setNextACSensSrc();
    }
    // **********************  end zero cross state machine code *************************
    // ***********************************************************************************

    // *************************************************************************************
    // ****** Power failover Handler/check for HV only *************************************
    // *************************************************************************************
    if(HV_POWER_FAILOVER)
        PowerFailOverTestAndHandle();

        
  }  // end if HV

  // ******************************************************************************************
  // RESET OF RPI LOGIC  /soft/hard
  // reboot is triggerd by loss of i2c traffic...
  //*********************************************************************************************
  /*   long delta = millis() - timeLastRebootRequest;
     if((delta) > REBOOT_WAITTIME_MS)  // dont check until 45 sec after the last reboot request
     {
        if(waitingForRPIBoot)
        {
           Serial.println("i2c Check started");
           waitingForRPIBoot = false;
        }
        rebootResolveI2C();  // <---------------------------------------------disable this line to remove this check
     }
     else
     {
         if(!waitingForRPIBoot)
         {
            waitingForRPIBoot = true;
            Serial.println("waiting for rpi boot");
         }
     }
  */
  // *****************************end reset / reboot logic of rpi  ***************


  //Serial.println(micros());

  // as of 9/14/17,  loop time on LV is 200uS

  loopcount++;                 //10000 == 25ms,
  if (loopcount >= 100000)     // 40000 == 100ms,
  {

    loopcount = 0;
    MeasureZero2TenInputs();  // this is non blocking. ...
    // NOTE ,  100000 is the slowest time allowed ,  for now,  other wise you will miss the clicks.
    MeasureWetDryContactInputs();  // this is non  blocking, but must be dont fast enough to pick up clicks.



    if (MEASURE_LOOPTIME)
    {
      Serial.print("looptime max: " );
      Serial.print(maxcycletime);

      Serial.print(" min: " );
      Serial.print(mincycletime);

      float avg = avgcycletime / (float)100000;

      Serial.print(" avg: " );
      Serial.println(avg);

      avgcycletime = 0;
      maxcycletime = 0;
      mincycletime = 10000.0;
    }

  }

  if (MEASURE_LOOPTIME)
  {
    long markmicros = micros();
    float deltatime = (float)(markmicros - test_last_loop_us);
    if (deltatime > maxcycletime)
      maxcycletime = deltatime;

    if (deltatime < mincycletime)
      mincycletime = deltatime;

    test_last_loop_us = markmicros;
    avgcycletime += deltatime;
  }

  //*****************************************************************
  // test code for reset countdown....loop, till reset.
  // *******************************************************************
  /*if(temp_softreset_countdown > 0)

    {
     temp_softreset_countdown--;
     if(temp_softreset_countdown == 0)
     {
       //softRebootRPI();
       hardRebootRPI();
     }
    }
  */
  // end countdown test ***************************
} // end main loop()



void MeasureLV_PWMCurrent()
{

  for (int i = 0; i < 8; i++)
  {
    pwm_sample_data[i] += analogRead(iMeas[i]);
  }
  pwmcurrentsamplecount++; //for div

  if (pwmcurrentsamplecount > 2000)
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

    //   Serial.print(" pwm sample data0: " );
    //  Serial.println(pwm_sample_data[0]);

    pwmcurrentsamplecount = 0;
    memset(pwm_sample_data, 0, sizeof(pwm_sample_data));

  }
}



// as of board rev D.  here is the pin/port mapping
// Sens0 - Aux0     
// Sens1 - Line 0              
// Sens2 - Aux1
// Sens3 - Line 1

void setNextACSensSrc()
{
  uint8_t nextsrc = 1;  //swizzle,  0 , 2, 1, 3,, ..0

  // if(hv_ac_sens_src == 1)
  // {
  //nextsrc = 3;
  // }
  //  else
  //   nextsrc = 1;


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
  switch (srcnum)
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
  if (bank == 0 || bank == 1)
  {
    // Serial.print("bank 0/1 freq: ");
    // Serial.println(freq);
    analogWriteFrequency(pwmPin_HVL[0], freq);  // these are on ftm 1,2,3
    analogWriteFrequency(pwmPin_HVL[1], freq);
    analogWriteFrequency(pwmPin_HVL[2], freq);
    analogWriteFrequency(pwmPin_HVL[3], freq);
  }
  else if (bank == 2 || bank == 3)              // NOTE ,  these are all on FTM 0
  {
    // Serial.print("bank 2/3 freq: ");
    //  Serial.println(freq);
    analogWriteFrequency(pwmPin_HVL[4], freq);


    //   if(delta > 0.10)
    // {
    //    Serial.print("**************big delta detected:  ");
    //     Serial.println(delta);
    // digitalWrite(plcOut[0], HIGH);
    //  delay(100);
    // digitalWrite(plcOut[0], LOW);

    // }

    // these are not needed becuase they are on the same ftm...
    //   analogWriteFrequency(pwmPin_HVL[5], freq);
    //  analogWriteFrequency(pwmPin_HVL[6], freq);
    //  analogWriteFrequency(pwmPin_HVL[7], freq);
  }
}


void setPWMState(uint8_t * pwmvals)
{
  Serial.print(" hv dim mode max : ");
  Serial.println(hv_dimmermode);


  Serial.print("pct |  pwmvals: ");




  for (int i = 0; i < 8; i++)
  {
    int pwmValue = pwmvals[(2 * i) + 1] * 256 + pwmvals[(2 * i) + 2];  // extract value from array that is sent over. from host.
    int pct = (pwmValue * 100) / 65535;  // convert it to a pct value.


    if (LV_HV == 0)   // 8/23/17,   only for lv now.
    {
      Serial.print(" | ");
      Serial.print(pct);
      Serial.print(" | ");
      analogWrite(pwmPin[i], pwmValue);
      pwmData[i] = pwmValue; // store value for ref
    }
    else  // HV
    {

      // 0 == phase, 1 0-10, dim,
      bool isPhaseDim = ((hv_dimmermode & (0x01 << i)) <= 0);

      int appliedpct = pct;
      if (isPhaseDim)
      {
        if (pct >= 80)
          appliedpct = 80;




        uint16_t dutyValueReverse = uint16_t(float(appliedpct * 0.01) * 65535); // rising edge. (forward)
        uint16_t dutyValueForward = uint16_t((1 - float(appliedpct * 0.01)) * 65535); // falling edge, (reverse)
        // Serial.print(" | ");
        //Serial.print(dutyValueReverse);
        // Serial.print(" | ");

        if (appliedpct >= 2) // if pct > 2 ,  turn on bias enable.
        {
          // if(((bias_enable >> i) & 0x01) <= 0)
          //{
          // if the bit is not set,  we need to delay the write.
          //Serial.println("delay pwm timestart now");
          //delaypwmupdate_ms = millis();
          //}
          bias_enable |= (0x01 << i);   // bias enable on
        }
        else
        {
          bias_enable &= ~(0x01 << i);  // bias enable off,
        }


        int val = pwmValue;
        if (((dimmerEdge >> i) & 0x01) > 0) // if forward
        {
          analogWrite(pwmPin_HVL[i], dutyValueForward);
          pwmData[i] = dutyValueForward;


          Serial.print("    ** ");
          Serial.print(appliedpct);
          Serial.print(" | ");
          Serial.print(dutyValueForward);

        }
        else  // if reverse
        {
          if (dutyValueReverse == 0)
            dutyValueReverse = 1;

          analogWrite(pwmPin_HVL[i], dutyValueReverse);
          pwmData[i] = dutyValueReverse;

          Serial.print("    ** ");
          Serial.print(appliedpct);
          Serial.print(" | ");
          Serial.print(dutyValueReverse);
        }


      } // end PHASE DIM
      else  // else if  ***********  0-10 volt out dimm mode,  then just write the pwm val.
      {
        Serial.print(" | ");
        Serial.print(pct);
        Serial.print(" | ");
        analogWrite(pwmPin[i], pwmValue);
        pwmData[i] = pwmValue; // store value for ref
      }


    }  // end HV
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

  timeStampMostRecentRx = millis();
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
  timeStampMostRecentRx = millis();
  // ------------------------------------------
  // Node GET Command received:
  // Teensy Tx Event (I2C data -> PI)
  // addr = command byte requested by Node i2c
  // ------------------------------------------

  if (pending_get_command == VERSION)
  {
    Wire.write(hw_info_buff, 4);
  }

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
  //uint8_t txbuff[4];
  Serial.println("got version request, sending now" );
  hw_info_buff[0] = 3;
  hw_info_buff[1] = LV_HV;
  hw_info_buff[2] = 0x02; //RELEASENUMBERMAJOR;
  hw_info_buff[3] = 0x06; //RELEASENUMBERMINOR;


  Wire.write(hw_info_buff, 4);
}

void MeasureZero2TenInputs()
{
  int tempdata = 0;
  //memset(zero2teninputstatus, 0, sizeof(zero2teninputstatus));
  //zero2teninputstatus[0] = 8;

  switch (current0_10_meas_index)
  {
    case 0:
      tempdata = analogRead(analogIn[0]);
      zero2teninputstatus[1] = (uint8_t)(tempdata >> 8) & 0xFF;
      zero2teninputstatus[2] = (uint8_t)(tempdata & 0xFF);
      break;
    case 1:
      tempdata = analogRead(analogIn[1]);
      zero2teninputstatus[3] = (uint8_t)(tempdata >> 8) & 0xFF;
      zero2teninputstatus[4] = (uint8_t)tempdata & 0xFF;
      break;
    case 2:
      tempdata = analogRead(analogIn[2]);
      zero2teninputstatus[5] = (uint8_t)(tempdata >> 8) & 0xFF;
      zero2teninputstatus[6] = (uint8_t)(tempdata & 0xFF);
      break;
    case 3:

      tempdata = analogRead(analogIn[3]);
      zero2teninputstatus[7] = (uint8_t)(tempdata >> 8) & 0xFF;
      zero2teninputstatus[8] = (uint8_t)(tempdata & 0xFF);
      break;
    default:
      break;
  }


  current0_10_meas_index++;
  if (current0_10_meas_index > 3)
  {
    current0_10_meas_index = 0;
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
      if ((dimmerEdge >> i) & 0x01)
      {
        edgeconfig = 0x0028;   // forward.
      }

      // NEW PORT MAP...   = {2, 3, 4, 29, 6, 9, 10, 5};
      switch (i)
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

void printdebug(char* str, int flag)
{
  if (flag)
  {
    Serial.print(str);
  }
}



// -------------------------------------------- Reboot logic -----------------------------


void softRebootRPI(void)
{
  // ------------------------------------------
  // Pull RPi pin12/GPIO-18 High for 1 sec
  // RPi pin12 has internal pulldown enabled
  // Informs RPi to run sudo shutdown -r now
  // ------------------------------------------delay(1000);
  Serial.println("Soft Reboot RPi!");
  Serial.println("PI_GPIO18_PIN12 toggle HIGH for 1 sec");
  digitalWrite(PI_GPIO18_PIN12, HIGH);
  delay(1000);

  Serial.println("PI_GPIO18_PIN12 returning to normal(LOW)");
  digitalWrite(PI_GPIO18_PIN12, LOW);
  Serial.println("Done");
}

void hardRebootRPI(void)
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
  long timeDeltaRx = millis() - timeStampMostRecentRx;
  // Serial.print("Main Loop: timeDeltaRx = ");
  // Serial.println(timeDeltaRx);

  if (timeDeltaRx > 1000 && softRebootTryNum < maxNumSoftRebootTries)
  {
    Serial.println("******************************");
    Serial.print("In Soft Reboot Loop ");
    Serial.print("Try # ");
    Serial.print(softRebootTryNum + 1);
    Serial.print("/");
    Serial.println(maxNumSoftRebootTries);
    Serial.println("******************************");

    softRebootRPI();
    //Serial.print("Waiting ");
    //Serial.print(REBOOT_WAITTIME_MS / 1000);
    //Serial.println("sec for RPi to boot");
    //delay(REBOOT_WAITTIME_MS);
    //Serial.print(REBOOT_WAITTIME_MS / 1000);  // By then an rx event should have occured
    //Serial.println(" seconds has elapsed");
    softRebootTryNum += 1;
    timeLastRebootRequest = millis();
  }

  if (timeDeltaRx > 1000 && softRebootTryNum == maxNumSoftRebootTries)
  {
    hardRebootRPI();
    softRebootTryNum = 0; // does it matter that this is after delay(45s)?
    //Serial.print("Waiting ");
    //Serial.print(REBOOT_WAITTIME_MS / 1000);
    //Serial.println("sec for RPi to boot");
    //delay(REBOOT_WAITTIME_MS);
    //Serial.print(REBOOT_WAITTIME_MS / 1000);  // By then an rx event should have occured
    //Serial.println(" seconds has elapsed");
    timeLastRebootRequest = millis();
  }
}


void setPWMBankto100pct(int bank)
{
  int startindex = 0;
  int endindex = 0;
  int i = 0;


  // **** NOTE  11/6/17  we cap out at 80% in reality, , so just set to 80 here. 
  
  int dutyValueReverse = int(float(80 * 0.01) * 65535); // rising edge. (forward)
  int dutyValueForward = int((1 - float(80 * 0.01)) * 65535); // falling edge, (reverse)

  if (bank == 0 || bank == 1)
  {
    startindex = 0;
    endindex = 4;
  }
  else if (bank == 2 || bank == 3)
  {
    startindex = 4;
    endindex = 8;
  }

  for (i = startindex; i < endindex; i++)
  {
    // int val = pwmValue;
    if (((dimmerEdge >> i) & 0x01) > 0) // if forward
    {
      analogWrite(pwmPin_HVL[i], dutyValueForward);
      pwmData[i] = dutyValueForward;
    }
    else  // if reverse
    {
      if (dutyValueReverse == 0)
        dutyValueReverse = 1;

      analogWrite(pwmPin_HVL[i], dutyValueReverse);
      pwmData[i] = dutyValueReverse;
    }
  }
  // analogWrite(pwmPin_HVL[0], level);
  // analogWrite(pwmPin_HVL[1], level);
  // analogWrite(pwmPin_HVL[2], level);
  // analogWrite(pwmPin_HVL[3], level);
  // }
  //else if (bank == 2 || bank == 3)
  //{
  // analogWrite(pwmPin_HVL[4], level);
  // analogWrite(pwmPin_HVL[5], level);
  // analogWrite(pwmPin_HVL[6], level);
  // analogWrite(pwmPin_HVL[7], level);
  //}


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

//********************************************* HV CURRENT MEAS (RMS) routines ****************
///********************************************************************************************
void MeasurePWMOutputCurrent()
{
  long currtime = micros();
  pwmcurrentstatus[0] = 16; // set first byte as messgae type.
  // if we are ready to calc avg.
  long sample_window_time_us = 1000000; // how long to samples for (total window)  1 sec
  long sample_slice_us = 10000; //time between sample. 10 ms

  if (LV_HV == 0)
  {
    sample_slice_us = 400;
    //sample_window_time_us = 1000000;  //X us sample window
    //sample_slice_us = 1000; //5ms; // X us between sample
    // every cycle,  take one sample from current channel .
    if ((currtime - lastsampletime) >= sample_slice_us)
    {
      pwm_sample_data[rms_channel_meas] += analogRead(iMeas[rms_channel_meas]);
      rms_channel_meas++;  // inc to next channel.
      lastsampletime = currtime;

      if (rms_channel_meas > 7)
      {
        rms_channel_meas = 0;
        pwmcurrentsamplecount++; // this is how many samples total we have taken. per channel.
      }

      if (pwmcurrentsamplecount > 150) // changed from 500)  v1.3 LV  // once we have X samples.
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
  }
  else
  {


    if ((currtime - rms_sample_start_time) >= sample_window_time_us)   // sample window is 1 sec
    {
      // Serial.print("********");
      // Serial.println(currtime);
      // Serial.println(rmsAccumHolder);
      long accum = rmsAccumHolder;
      float avg = (float)accum / (float)num_samples_taken;

      uint16_t counts = 0;
      //if (LV_HV == 1)  // if HV do rms calc
      // {
      double powerSumation;

      // Remove DC component and calculate square
      for ( int i = 0; i < num_samples_taken; i++ )
      {
        float temp = (float)sampleHolder[i] - avg;
        powerSumation += pow ( temp, 2);
      }

      // Calculate RMS
      long powerMean0 = powerSumation / num_samples_taken; //num_samples_taken;
      double RMS = sqrt(powerMean0);

      counts = (uint16_t)RMS;  // convert the float to a uint16,
      //  }
      // else if (LV_HV == 0) // if lv
      //{
      //  counts = (uint16_t)avg;  // just take avg.
      // }


      pwmcurrentstatus[(rms_channel_meas * 2) + 1] = counts / 256; //(counts >> 8) & 0xFF;   //msb
      pwmcurrentstatus[(rms_channel_meas * 2) + 2] = counts & 0xFF; //(uint8_t)RMS;  // lsb, ;

      // for debug  ..**********************************
      //Serial.print("samplecount:   min max counts  avg  counts:  ");

      // Serial.print(num_samples_taken);
      // Serial.print("   :  ");
      // Serial.print(samplemin);
      //Serial.print("  ");
      //Serial.print(samplemax);
      //Serial.print("  ");
      // Serial.println(counts);
      // Serial.print("  ");
      // Serial.println(avg);
      // Serial.print("  ");

      // Serial.print("   power mean  ");
      // Serial.print(powerMean0);

      // Serial.print("   RMS :  ");
      // Serial.println(RMS);
      // *********************************************
      // zero and reset for next measumrent.
      rmsAccumHolder = 0;
      num_samples_taken = 0;
      rms_sample_start_time = currtime;
      samplemin = 0xFFFF;
      samplemax = 0;

      memset(sampleHolder, 0, sizeof(sampleHolder));
      rms_channel_meas++;  // inc to next channel.
      if (rms_channel_meas > 7)
      {
        rms_channel_meas = 0;
      }

    }
    else if ((currtime - lastsampletime) >= sample_slice_us) // take a sample every 10 ms
    {
      // Serial.print("^^^^^^^^");
      // Serial.println(currtime);
      long temp = analogRead(iMeas[rms_channel_meas]);

      rmsAccumHolder += temp;
      if (LV_HV == 1)  // if HV do rms calc
        sampleHolder[num_samples_taken] = temp;


      lastsampletime = currtime;
      num_samples_taken++;

      if (temp > samplemax)
        samplemax = temp;

      if (temp < samplemin)
        samplemin = temp;
      // Serial.println("sample taken");
    }

  }

}

//************************************ Power Failover Handler *****************
/**************************************************************************************************************/
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
        setPWMBankto100pct(0);
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
        setPWMBankto100pct(2);
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

