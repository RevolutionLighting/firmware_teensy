void shiftRegisterWriteByte(int output)
{
  Serial.println("shiftRegisterWriteByte() called");
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
  Serial.println("shiftRegisterWriteWord16() called");
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
  Serial.println("shiftRegisterWriteWord24 called");
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
