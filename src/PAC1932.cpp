/******************************************************************************
PCA1932.cpp
Library for PCA1932 ADC
Bobby Schulz 
11/17/2020

///////////////////FIX///////////////////////////////////
MCP3421 is a 18 bit ADC, PGA, and various modes of resolution operation in order to facilitate various data rates

"Simplicity is prerquisite for reliability"
-Edsger W. Dijkstra

Distributed as-is; no warranty is given.
******************************************************************************/

#include <Arduino.h>
#include <PAC1932.h>
#include <Wire.h>



PAC1932::PAC1932(float _R1, float _R2, int _ADR)  //Set address and CSR values [mOhms]
{
  ADR = _ADR; 
  R[0] = _R1;
  R[1] = _R2;
  // Wire.begin();  
}

// PAC1932::PAC1932(void)  //Use default address
// {
//   ADR = CSA_ADR; //Default address
// }

bool PAC1932::begin() //Initalize system 
{
  Wire.begin();  
  //Setup device for default operation
  Wire.beginTransmission(ADR);
  Wire.write(0x00);
  return Wire.endTransmission(); 
  // SetConfig(C1RA, 0x00); //No averaging on CH1 bus measurment 
  // SetConfig(C2RA, 0x00); //No averaging on CH2 bus measurment 
  // SetConfig(C1RS, 0b11); //11 bit sample on CH1 bus
  // SetConfig(C2RS, 0b11); //11 bit sample on CH2 bus
  // SetConfig(C1CSS, 0b101); //11 bit sense, min sample time, on CH1 sense
  // SetConfig(C2CSS, 0b101); //11 bit sense, min sample time, on CH2 sense
  // SetConfig(C1SR, 0b11); //Set for 80mV FSR on CH1
  // SetConfig(C2SR, 0b11); //Set for 80mV FSR on CH2
  // SetConfig(C1SA, 0b00); //No averaging on CH1 sense measurment 
  // SetConfig(C2SA, 0b00); //No averaging on CH2 sense measurment 
}

float PAC1932::GetBusVoltage(uint8_t Unit, bool Avg) //Have option to take vaerage of last 8 samples 
{
  bool VoltageDir = GetVoltageDirection(Unit);
  if(VoltageDir) return float(int16_t(GetVoltageRaw(BUS1 + Unit, Avg)))*(32.0/32768.0); //Return result in mV, cast to signed
  else return float(GetVoltageRaw(BUS1 + Unit, Avg))*(32.0/65536.0); //Return result in mV
  // return float(GetVoltageRaw(BUS1 + Unit, Avg))*(0.488); //Return result in mV 
}

float PAC1932::GetSenseVoltage(int Unit, bool Avg) //Have option to take vaerage of last 8 samples //FIX! encorporate bi-directionality 
{
  return float(GetVoltageRaw(SENSE1 + Unit, Avg))*(1.525878906); //Return result in uV
  // //FSR = 10, 20, 40, 80mV
  // float FSR = 80; //Set to 80 by default
  // unsigned int Den = 2047; //Set to 2047 by default

  // if(Unit == CH1) {
  //   //FSR = 10, 20, 40, 80mV
  //   FSR = (2 << GetConfig(C1SR))*10;
  //   Den = 6 + GetConfig(C1CSS);
  // }
  // if(Unit == CH2) {
  //   //FSR = 10, 20, 40, 80mV
  //   FSR = (2 << GetConfig(C2SR))*10;
  //   Den = 6 + GetConfig(C2CSS);
  // }

  // if(Den > 11) Den = 11; //FIX! Make cleaner 
  // Den = (2 << Den) - 1;
  // float FSC = FSR; //FSR = mV

  // Wire.beginTransmission(ADR);
  // Wire.write(SENSE1 + 2*Unit); //Write to bus voltage reg+offset
  // uint8_t Error = Wire.endTransmission();

  // Wire.requestFrom(ADR, 2);
  // int Data1 = Wire.read(); //Read out bytes
  // int Data2 = Wire.read();

  // // if((Data2 & 0x0F) == 0x0F) return 0; //Check if low bits (fixed at 0) are set, if so, reading is bad, return 0 //FIX??
  // int16_t VSense = ((Data1 << 4) | Data2 >> 4); //IMPORTANT! Must be int16_t, NOT int! Should be equitable, but if use int, it gets treated as unsigned later. Not very cash money. 
  
  // if((VSense & 0x800) == 0x800) VSense = VSense | 0xF000; //If sign bit it set, pad left FIX!
  // float I = FSC*float(VSense)/float(Den);
  // if(Error == 0) return I; //Return in mV FIX??
  // else return 0; //If I2C state is not good, return 0
}

float PAC1932::GetCurrent(int Unit, bool Avg) //Have option to take vaerage of last 8 samples 
{
  bool CurrentDir = GetCurrentDirection(Unit);
  uint16_t VSense = GetVoltageRaw(SENSE1 + Unit, Avg);
  float FSC = 100.0/R[Unit]; //Calculate the full scale current, [Amps] using the defined resistor value
  float Current = 0; 
  if(CurrentDir) Current = FSC*(int16_t(VSense)/(32768.0)); //If set for bi-directional, cast to signed value
  else Current = FSC*(VSense/(65536.0));

  return Current*1000.0; //return mA

  // //FSR = 10, 20, 40, 80mV
  // float FSR = 80; //Set to 80 by default
  // unsigned int Den = 2047; //Set to 2047 by default

  // if(Unit == CH1) {
  //   //FSR = 10, 20, 40, 80mV
  //   FSR = (2 << GetConfig(C1SR))*10;
  //   Den = 6 + GetConfig(C1CSS);
  // }
  // if(Unit == CH2) {
  //   //FSR = 10, 20, 40, 80mV
  //   FSR = (2 << GetConfig(C2SR))*10;
  //   Den = 6 + GetConfig(C2CSS);
  // }

  // if(Den > 11) Den = 11; //FIX! Make cleaner 
  // Den = (2 << Den) - 1;
  // float FSC = FSR/R; //FSR = mV, R = mOhms

  // Wire.beginTransmission(ADR);
  // Wire.write(SENSE1 + 2*Unit); //Write to bus voltage reg+offset
  // uint8_t Error = Wire.endTransmission();

  // Wire.requestFrom(ADR, 2);
  // int Data1 = Wire.read(); //Read out bytes
  // int Data2 = Wire.read();

  // // if((Data2 & 0x0F) == 0x0F) return 0; //Check if low bits (fixed at 0) are set, if so, reading is bad, return 0 //FIX??
  // int16_t VSense = ((Data1 << 4) | Data2 >> 4); //IMPORTANT! Must be int16_t, NOT int! Should be equitable, but if use int, it gets treated as unsigned later. Not very cash money. 
  
  // if((VSense & 0x800) == 0x800) VSense = VSense | 0xF000; //If sign bit it set, pad left FIX!
  // float I = FSC*float(VSense)/float(Den);
  // if(Error == 0) return I*1000.0; //Return in mA FIX??
  // else return 0; //If I2C state is not good, return 0
  
}

int PAC1932::GetConfig(Config Value) //Return the value of the given configuration 
{
  // uint8_t Reg = Value >> 8; //Pull out register
  // uint8_t Offset = (Value & 0x0F); //Grab offset value 
  // uint8_t Mask = (Value >> 4) & 0x0F; //Grab config mask

  // Wire.beginTransmission(ADR);
  // Wire.write(Reg);
  // Wire.endTransmission(); //FIX! Add error condition 

  // Wire.requestFrom(ADR, 1);
  // int Data = Wire.read();

  // return (Data >> Offset) & Mask;
}

int PAC1932::SetConfig(Config Value, uint8_t NewVal) //Set the value of the given configuration 
{
  // uint8_t Reg = (Value >> 8); //Pull out register
  // uint8_t Offset = (Value & 0x0F); //Grab offset value 
  // uint8_t Mask = (Value >> 4) & 0x0F; //Grab config mask

  // Wire.beginTransmission(ADR);
  // Wire.write(Reg);
  // Wire.endTransmission();

  // Wire.requestFrom(ADR, 1);
  // uint8_t PrevVal = Wire.read(); //Grab current state of register
  // PrevVal = PrevVal & ~(Mask << Offset); //Clear desired bits, preserve rest

  // uint8_t WriteVal = (NewVal << Offset) | PrevVal; //Shift desired value into correct position, or with other bits

  // Wire.beginTransmission(ADR);
  // Wire.write(Reg);
  // Wire.write(WriteVal); //Write adjusted value
  // return Wire.endTransmission(); //Return error condition
}

void PAC1932::SetVoltageDirection(uint8_t Unit, bool Direction)
{
  uint8_t RegTemp = ReadByte(NEG_PWR, ADR);
  if(Direction) {
    WriteByte(NEG_PWR, (RegTemp | (0x08 >> Unit)), ADR); //Write modified value back
  }
  else WriteByte(NEG_PWR, (RegTemp & ~(0x08 >> Unit)), ADR); //Clear desired bit, write back
}

void PAC1932::SetCurrentDirection(uint8_t Unit, bool Direction)
{
  uint8_t RegTemp = ReadByte(NEG_PWR, ADR);
  if(Direction) {
    WriteByte(NEG_PWR, (RegTemp | (0x80 >> Unit)), ADR); //Write modified value back
  }
  else WriteByte(NEG_PWR, (RegTemp & ~(0x80 >> Unit)), ADR); //Clear desired bit, write back
}

bool PAC1932::GetVoltageDirection(uint8_t Unit)
{
  uint8_t RegTemp = ReadByte(NEG_PWR, ADR);
  return (RegTemp >> (3 - Unit)) & 0x01; //Return Specified bit
}

bool PAC1932::GetCurrentDirection(uint8_t Unit)
{
  uint8_t RegTemp = ReadByte(NEG_PWR, ADR);
  return (RegTemp >> (7 - Unit)) & 0x01; //Return Specified bit
}

int64_t PAC1932::ReadBlock(uint8_t Unit)
{
  Wire.beginTransmission(ADR);
  Wire.write(0x03 + Unit); //Write value to point to block
  Wire.endTransmission();

  Wire.requestFrom(ADR, 6);
  // int8_t[6] Data = {0};
  int64_t Val = 0; //Concatonated result 
  for(int i = 0; i < 6; i++){
    // Data[i] = Wire.read(); //Read im each data byte
    Val = Val | (Wire.read() << i); //FIX! Deal with sign! 
  }

  return Val; 

}

uint32_t PAC1932::ReadCount()
{
  // Update(); //Update  accumulators 
  // delay(2); //DEBUG!
  Wire.beginTransmission(ADR);
  Wire.write(COUNT_REG); //Write value to point to block
  Wire.endTransmission();

  Wire.requestFrom(ADR, 3, false);
  while(Wire.available() < 3); //FIX! Add timeout 
  // int8_t[6] Data = {0};
  uint32_t Data = 0; //Concatonated result 
  uint32_t Val = 0; //Used to hold each I2C byte to force bit shift behavior 
  for(int i = 0; i < 3; i++){
    // Data[i] = Wire.read(); //Read im each data byte
    Val = Wire.read();
    Data = Data | (Val << (2 - i)*8); //FIX!
  }

  return Data; 

}

void PAC1932::SetFrequency(Frequency SampleRate)
{ 
  uint8_t Temp = ReadByte(CTRL_REG, ADR); //Grab current value from the control register 
  Temp = Temp & 0x3F; //Clear sample bits
  Temp = Temp | (SampleRate << 6); //Set new sample rate
  WriteByte(CTRL_REG, Temp, ADR); //Write updated value back
  // Update(); //Call non-destructive update to enforce new settings //FIX! Keep?
  // delay(1); //Wait for new values 
}

//Check is overflow has occoured before last reset
bool PAC1932::TestOverflow()
{
  uint8_t Control = ReadByte(CTRL_REG, ADR); //Grab control reg
  if(Control & 0x01) return true;
  else return false;
}

float PAC1932::GetPowerAvg(int Unit)
{
  uint32_t NumPoints = ReadCount(); //Grab the number of points taken
  uint64_t Val = ReadAccBlock(Unit, ADR); //Grab the desired accumulator block
  float ValAvg = float(Val)/float(NumPoints); //Normalize for quantity acumulated //FIX! Check for overflow??
  float FSR = (3200.0/R[Unit]); //Find power FSR based on resistance of given sense resistor 
  float P_Prop = float(ValAvg)/(134217728.0); //Divide by 2^27 to normalize //FIX! add support for negative values 
  float PowerAvg = P_Prop*FSR; //Calculate average power 
  return PowerAvg; //DEBUG!

}

uint16_t PAC1932::ReadWord(uint8_t Reg, uint8_t Adr)  //Send command value, returns entire 16 bit word
{
  Wire.beginTransmission(Adr);
  Wire.write(Reg);
  uint8_t Error = Wire.endTransmission(); //Store Error

  Wire.requestFrom(Adr, 2, true);
  while(Wire.available() < 2); //FIX! Add timeout 
  uint8_t ByteHigh = Wire.read();  //Read in high and low bytes (big endian)
  uint8_t ByteLow = Wire.read();

  return ((ByteHigh << 8) | ByteLow); //DEBUG!
}

uint8_t PAC1932::ReadByte(uint8_t Reg, uint8_t Adr)  //Send command value, returns byte
{
  Wire.beginTransmission(Adr);
  Wire.write(Reg);
  uint8_t Error = Wire.endTransmission(); //Store Error

  Wire.requestFrom(Adr, 1, true);
  while(Wire.available() < 1); //FIX! Add timeout 
  return Wire.read();  //Read single byte
  // uint8_t ByteLow = Wire.read();

  // return ((ByteHigh << 8) | ByteLow); //DEBUG!
}

uint64_t PAC1932::ReadAccBlock(uint8_t Unit, uint8_t Adr)  //Read 48 bit accumulator values of the desired channel 
{
  Wire.beginTransmission(Adr);
  Wire.write(ACCUMULATOR_REG_0 + Unit); //Point to accumulator block plus offset
  uint8_t Error = Wire.endTransmission(); //Store Error

  // uint8_t Data[6] = {0}; //Instatiate data array to store block in
  uint64_t Data = 0; //Concatonated data
  Wire.requestFrom(Adr, 6, true);
  while(Wire.available() < 6); //FIX! Add timeout 
  // return Wire.read();  //Read single byte
  uint64_t Val = 0; //Used to hold I2C reads
  for(int i = 0; i < BLOCK_LEN; i++) { //Drop entire block into Data array
    Val = Wire.read();
    Data = Data | (Val << (BLOCK_LEN - i - 1)*8); //Shift and accumulate each byte
    // Serial.println((BLOCK_LEN - i - 1)*8); //DEBUG!
    // Serial.println(Val, HEX);
  }

  //FIX! Deal with signed results
  //FIX! Check for errors before return

  // uint8_t ByteLow = Wire.read();

  // return ((ByteHigh << 8) | ByteLow); //DEBUG!
  return Data; //Return concatonated value
}

uint8_t PAC1932::WriteByte(uint8_t Reg, uint8_t Data, uint8_t Adr)
{
  Wire.beginTransmission(Adr);
  Wire.write(Reg);
  Wire.write(Data);
  return Wire.endTransmission();
}

uint16_t PAC1932::GetVoltageRaw(uint8_t Reg, bool Avg)
{
  // Update(); //DEBUG!
  // delay(1); //DEBUG!
  uint8_t Offset = 0; //Offset used to select for average
  if(Avg) Offset = 0x08; //Offset for average registers 
  return ReadWord(Offset + Reg, ADR);
}

// float PAC1932::GetBusVoltage(uint8_t Unit)
// {
//   return float(GetBusVoltageRaw(Unit))*(0.488); //Return result in mV
// }

uint8_t PAC1932::Update(uint8_t Clear)
{
  uint8_t Reg = 0x1F; //Default to refesh, no clear
  if(!Clear) Reg = 0x1F;
  else if(Clear) Reg = 0x00;

  // Wire.beginTransmission(ADR);
  // Wire.write(Reg);
  // Wire.write(0x00); //Initilize a clear
  // return Wire.endTransmission();

  Wire.beginTransmission(ADR);
  Wire.write(Reg);
  // Wire.write(0x00); //Initilize a clear
  uint8_t Error = Wire.endTransmission();
  delay(2); //FIX! Find exact delay required 
}









