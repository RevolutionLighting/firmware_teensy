

void setHV_AC_SENS_SRC(uint8_t src)
{
    // src = 0 - 3 ,  
    hv_ac_sens_src = src;
  // Serial.print("ac source updated  ");
  // Serial.println(hv_ac_sens_src);
   last_hv_ac_sens_src_switch_mills = millis();
    updateHVSR();

   
}

void updateHVSR()
{
    if (LV_HV==1)
    {
        int src_nib = 0;
        if(hv_psource0)
            src_nib |= 0x01;
         if(hv_psource1)
            src_nib |= 0x02;


        
         // test. 
           
        // make MSB out of remainder parts.
      //  int shift = hv_ac_sens_src + 2;
      //  uint8_t MSB = (0x01 << shift) | src_nib;  //active high way 

       
        //active low on ac sens I think, 
         uint8_t sensmask = 0x0F;
         sensmask &= ~(0x01 << hv_ac_sens_src); // clear bit,
        uint8_t MSB = (sensmask << 2) | src_nib;  //active low way, 
         // ************ END ACTIVE low way *******************

        
        int outputWord = MSB << 8 | hv_dimmerrelaystate;
       
      
       
       // shiftRegisterWriteWord16(outputWord);            // < --------------------- bring in when sr hw / relays are in place   ---------------- 

          int temp = (int)bias_enable << 16;
          int temp2 = temp | outputWord;
     //   Serial.println("-----------------------------");
      //  Serial.print("Update HVSR with 24 bit value: ");
     //   Serial.println(temp2,HEX);
     //   Serial.println("-----------------------------");
        shiftRegisterWriteWord24( temp2); //  // <<<<<<<<<<<<<<<<<<<------ 9/29/16,  update -------------------------------->  
    }
}


// for reading,  clock,  then copy bit value to local,  then next...
// MSB,   
byte shiftRegisterReadByte()
{
 // Serial.println("shiftRegisterReadByte() called");
// updateHVSR(dimmerEdge, dimmerRelayState);  //    NGP removed 9/7/16,  not sure why this is there... maybe to update state vars. ???, 
 
  byte i=0;
  byte val= 0x00;
  digitalWriteFast(SR_CLK,LOW);  // clock high
  digitalWriteFast(SR_LAT,LOW);  // latch low
  delayMicroseconds(5);
  digitalWriteFast(SR_LAT,HIGH);  // latch high
  
  for (i=0;i<8;i++)
  { 
    long bitVal = digitalReadFast(SR_DAT_IN);  // notice its  a read,  and its uding the data in line, k
    val |= (bitVal << ((8-1) - i));
    digitalWriteFast(SR_CLK,LOW);
    delayMicroseconds(1);
    digitalWriteFast(SR_CLK,HIGH);
    delayMicroseconds(1);
   // Serial.print(bitVal); 
   // Serial.print("   |"); 
  }   
  // Serial.println("   done"); 
   return val;
}



void shiftRegisterWriteByte(int output)
{
//  Serial.println("shiftRegisterWriteByte() called");
  byte i=0;
  for (i=0;i<8;i++)
  {
    if((output&0x0080)==0x0080) digitalWriteFast(SR_DAT,HIGH);
    else digitalWriteFast(SR_DAT,LOW);
    delayMicroseconds(10);
    digitalWriteFast(SR_CLK,HIGH);
    delayMicroseconds(10);
    digitalWriteFast(SR_CLK,LOW);
    delayMicroseconds(10);
    output<<=1;
  }
  digitalWriteFast(SR_LAT,HIGH);
  delayMicroseconds(10);
  digitalWriteFast(SR_LAT,LOW);
}

/*
void shiftRegisterWriteWord16(int output)
{
 // Serial.println("shiftRegisterWriteWord16() called");
  byte i=0;
  for (i=0;i<16;i++)
  {
    if((output&0x8000)==0x8000) digitalWriteFast(SR_DAT,HIGH);
    else digitalWriteFast(SR_DAT,LOW);
    delayMicroseconds(10);
    digitalWriteFast(SR_CLK,HIGH);
    delayMicroseconds(10);
    digitalWriteFast(SR_CLK,LOW);
    delayMicroseconds(10);
    output<<=1;
  }
  digitalWriteFast(SR_LAT,HIGH);
  delayMicroseconds(10);
  digitalWriteFast(SR_LAT,LOW);
  
}
*/

void shiftRegisterWriteWord16(int output)
{
 int delayus = 2;
 // Serial.println("shiftRegisterWriteWord16() called");
  byte i=0;
  for (i=0;i<16;i++)
  {
    if((output&0x8000)==0x8000) digitalWriteFast(SR_DAT,HIGH);
    else digitalWriteFast(SR_DAT,LOW);
    delayMicroseconds(delayus);
    digitalWriteFast(SR_CLK,HIGH);
    delayMicroseconds(delayus);
    digitalWriteFast(SR_CLK,LOW);
    delayMicroseconds(delayus);
    output<<=1;
  }
  digitalWriteFast(SR_LAT,HIGH);
  delayMicroseconds(10);
  digitalWriteFast(SR_LAT,LOW);
  
}


/*void shiftRegisterWriteWord16_v2(int output)
{
 // Serial.println("shiftRegisterWriteWord16() called");
  byte i=0;
  uint8_t bitout;
  
   digitalWriteFast(SR_LAT,LOW);
  for (i=0;i<16;i++)
  {
     bitout = (output >> i) & 0x01;
     
    shiftOut(SR_DAT, SR_CLK, LSBFIRST, bitout);
   // output<<=1;
  }
  digitalWriteFast(SR_LAT,HIGH);
 
 
}*/

void shiftRegisterWriteWord24(int output)
{
 // Serial.println("shiftRegisterWriteWord24 called");
  int delayus = 2;
  byte i=0;
  for (i=0;i<24;i++)
  {
    if((output&0x800000)==0x800000) digitalWriteFast(SR_DAT,HIGH);
    else digitalWriteFast(SR_DAT,LOW);
    delayMicroseconds(delayus);
    digitalWriteFast(SR_CLK,HIGH);
    delayMicroseconds(delayus);
    digitalWriteFast(SR_CLK,LOW);
    delayMicroseconds(delayus);
    output<<=1;
  }
  digitalWriteFast(SR_LAT,HIGH);
  delayMicroseconds(delayus);
  digitalWriteFast(SR_LAT,LOW);
}
