

void updateHVSR(int dimmeredge, uint8_t dimmerrelaystate)
{
    if (LV_HV==0)
    {   
        // LV Board
        // One 8-bit shift register
        // constructing serial stream being send to shift registers
        //               (mask off bottom 4 bits
        int outputWord = ((dimmeredge&0xF0)<<8)+((dimmerrelaystate&0xF0)<<4)+((dimmeredge&0x0F)<<4)+(dimmerrelaystate&0x0F);
        // int outputWord = ((dimmerEdge&0x0F)<<12)+((dimmerRelayState&0x0F)<<8)+((dimmerEdge&0xF0))+((dimmerRelayState&0xF0)>>4);
       // Serial.println("");
       // Serial.print("Update HVSR with 16 bit value ");
       // Serial.println(outputWord,HEX);
        shiftRegisterWriteWord16(outputWord);
    }
    if (LV_HV==1)
    {
//        int outputWord = ((dimmerEdge&0xF0)<<8)+((dimmerRelayState&0xF0)<<4)+((dimmerEdge&0x0F)<<4)+(dimmerRelayState&0x0F);
        int outputWord = (powerfailovermask<<16)+((dimmeredge&0xF0)<<8)+((dimmerrelaystate&0xF0)<<4)+((dimmeredge&0x0F)<<4)+((dimmerrelaystate&0x0F));
       // Serial.println("");
      //  Serial.print("Update HVSR with 24 bit value ");
      //  Serial.println(outputWord,HEX);
        shiftRegisterWriteWord24(outputWord);
    }
}


// for reading,  clock,  then copy bit value to local,  then next...
// MSB,   
byte shiftRegisterReadByte()
{
 // Serial.println("shiftRegisterReadByte() called");
 updateHVSR(dimmerEdge, dimmerRelayState);
 
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

void shiftRegisterWriteWord24(int output)
{
 // Serial.println("shiftRegisterWriteWord24 called");
  byte i=0;
  for (i=0;i<24;i++)
  {
    if((output&0x800000)==0x800000) digitalWriteFast(SR_DAT,HIGH);
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
