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
#include "PAC1934.h"
#include <Wire.h>



PAC1934::PAC1934(float _R1, float _R2, float _R3, float _R4, int _ADR)  //Set address and CSR values [mOhms]
{
  ADR = _ADR; 
  R[0] = _R1;
  R[1] = _R2;
  R[2] = _R3;
  R[3] = _R4;
  // Wire.begin();  
}

// PAC1934::PAC1934(void)  //Use default address
// {
//   ADR = CSA_ADR; //Default address
// }

bool PAC1934::begin() //Initalize system 
{
  #if defined(ARDUINO) && ARDUINO >= 100 
      Wire.begin();
  #elif defined(PARTICLE)
      if(!Wire.isEnabled()) Wire.begin(); //Only initialize I2C if not done already //INCLUDE FOR USE WITH PARTICLE 
  #endif 
  //Setup device for default operation
  // Wire.beginTransmission(ADR);
  // Wire.write(0xFD);
  // Wire.endTransmission(); //Store error for status update
  // Wire.read();
  uint8_t dummy;
  int val = readByte(PRODUCT_ID_REG, ADR, dummy);
  Serial.print("CSA READING: "); //DEBUG!
  Serial.println(val, HEX);
  // WriteByte(0x1C, 0x70, ADR); //DEBUG! Turn off measurment of all channels but channel 1
  if(val != 0x5B) return false; //Fail if ID reg does not match expected for PAC1934
  else { 
    writeByte(CTRL_REG, 0x0A, ADR); //Turn on ALERT on overflow //FIX??
    return true; //Pass initialization
  }
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

float PAC1934::getBusVoltage(uint8_t Unit, bool Avg) //Have option to take average of last 8 samples 
{
  bool dummy;
  return getBusVoltage(Unit, Avg, dummy); //Pass dummy to ignore value check
}

float PAC1934::getBusVoltage(uint8_t Unit, bool Avg, bool &Stat) //Have option to take average of last 8 samples 
{
  bool VoltageDir = getVoltageDirection(Unit);
  uint8_t error = 0;
  float val = 0; 
  if(VoltageDir) val = float(int16_t(getVoltageRaw(BUS1 + Unit, Avg, error)))*(32.0/32768.0); //Return result in V, cast to signed
  else val = float(getVoltageRaw(BUS1 + Unit, Avg, error))*(32.0/65536.0); //Return result in V

  if(error == 0) {
    Stat = false; //Clear error flag
    return val; //Return value as normal [V]
  }
  else {
    Stat = true; //Set error flag
    return 0; //Return empty value
  }
  // return float(GetVoltageRaw(BUS1 + Unit, Avg))*(0.488); //Return result in V 
}

float PAC1934::getSenseVoltage(int Unit, bool Avg) //Have option to take vaerage of last 8 samples //FIX! encorporate bi-directionality 
{
  bool dummy;
  return getSenseVoltage(Unit, Avg, dummy); //Return result in uV
}


float PAC1934::getSenseVoltage(int Unit, bool Avg, bool &Stat) //Have option to take vaerage of last 8 samples //FIX! encorporate bi-directionality 
{
  uint8_t error = 0;
  float val = float(getVoltageRaw(SENSE1 + Unit, Avg, error))*(1.525878906); //Get result in uV
  if(error == 0) {
    Stat = false; //Clear error flag
    return val; //Return value as normal [uV]
  }
  else {
    Stat = true; //Set error flag
    return 0; //Return empty value
  }
}

float PAC1934::getCurrent(int Unit, bool Avg)
{
  bool dummy;
  return getCurrent(Unit, Avg, dummy); //Pass dummy to ignore value check
}

float PAC1934::getCurrent(int Unit, bool Avg, bool &Stat) //Have option to take vaerage of last 8 samples 
{
  bool CurrentDir = getCurrentDirection(Unit);
  uint8_t error = 0;
  uint16_t VSense = getVoltageRaw(SENSE1 + Unit, Avg, error);
  if(error == 0) {
    Stat = false; //Clear error flag 
    float FSC = 100.0/R[Unit]; //Calculate the full scale current, [Amps] using the defined resistor value
    float Current = 0; 
    if(CurrentDir) Current = FSC*(int16_t(VSense)/(32768.0)); //If set for bi-directional, cast to signed value
    else Current = FSC*(VSense/(65536.0));

    return Current*1000.0; //return mA
  }
  else {
    Stat = true; //Set error flag
    return 0;
  }
}

int PAC1934::getFrequency()
{
  uint8_t dummy;
  uint8_t config = readByte(CTRL_REG, ADR, dummy); //Grab config value
  uint8_t freqVal = (config >> 6); //Grab upper 2 bits of config, where sample rate is configured 
  switch(freqVal) {
    case 0b11:
      return 8;
      break;
    
    case 0b10:
      return 64;
      break;
    
    case 0b01:
      return 256;
      break;
    
    case 0b00:
      return 1024;
      break;
  }
  return 0; //Return error otherwise 
}

int PAC1934::getConfig(Config Value) //Return the value of the given configuration 
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
  return 0; //DEBUG!
}

int PAC1934::setConfig(Config Value, uint8_t NewVal) //Set the value of the given configuration 
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
  return 0; //DEBUG!
}

void PAC1934::setVoltageDirection(uint8_t Unit, bool Direction)
{
  uint8_t dummy = 0;
  uint8_t RegTemp = readByte(NEG_PWR, ADR, dummy);
  uint8_t Error = 0; //Used to track errors from I2C communication 
  if(Direction) {
    Error = writeByte(NEG_PWR, (RegTemp | (0x08 >> Unit)), ADR); //Write modified value back, check error
  }
  else Error = writeByte(NEG_PWR, (RegTemp & ~(0x08 >> Unit)), ADR); //Clear desired bit, write back, check error
  // if(Error == 0) DirV[Unit] = Direction; //If write is sucessful, update direction value 
  //FIX! Return error? 
}

void PAC1934::setCurrentDirection(uint8_t Unit, bool Direction)
{
  uint8_t dummy = 0;
  uint8_t RegTemp = readByte(NEG_PWR, ADR, dummy);
  uint8_t Error = 0;
  if(Direction) {
    Error = writeByte(NEG_PWR, (RegTemp | (0x80 >> Unit)), ADR); //Write modified value back, check error
  }
  else Error = writeByte(NEG_PWR, (RegTemp & ~(0x80 >> Unit)), ADR); //Clear desired bit, write back, check error
  // if(Error == 0) DirI[Unit] = Direction; //If write is sucessful, update direction value 
  //FIX! Return error?
}

bool PAC1934::getVoltageDirection(uint8_t Unit)
{
  uint8_t dummy = 0;
  uint8_t RegTemp = readByte(NEG_PWR, ADR, dummy);
  return (RegTemp >> (3 - Unit)) & 0x01; //Return Specified bit
}

bool PAC1934::getCurrentDirection(uint8_t Unit)
{
  uint8_t dummy = 0;
  uint8_t RegTemp = readByte(NEG_PWR, ADR, dummy);
  return (RegTemp >> (7 - Unit)) & 0x01; //Return Specified bit
}

int64_t PAC1934::readBlock(uint8_t Unit, uint8_t &Err)
{
  Wire.beginTransmission(ADR);
  Wire.write(0x03 + Unit); //Write value to point to block
  Err = Wire.endTransmission();

  if(Err == 0) {
    unsigned long LocalTime = millis();
    Wire.requestFrom(ADR, 6, false);
    while(Wire.available() < COUNT_LEN && (millis() - LocalTime) < I2C_Timeout);
    // int8_t[6] Data = {0};
    int64_t Val = 0; //Concatonated result 
    for(int i = 5; i >= 0; i--){
      // Data[i] = Wire.read(); //Read im each data byte
      Val = Val | (Wire.read() << (i)*8); //FIX! Deal with sign! 
    }

    return Val; 
  }
  else return 0;

}

uint32_t PAC1934::readCount(uint8_t &Err)
{
  // Update(); //Update  accumulators 
  // delay(2); //DEBUG!
  Wire.beginTransmission(ADR);
  Wire.write(COUNT_REG); //Write value to point to block
  Err = Wire.endTransmission();

  if(Err == 0) {
    unsigned long LocalTime = millis();
    Wire.requestFrom(ADR, COUNT_LEN, false);
    while(Wire.available() < COUNT_LEN && (millis() - LocalTime) < I2C_Timeout);
    // int8_t[6] Data = {0};
    uint32_t Data = 0; //Concatonated result 
    uint32_t Val = 0; //Used to hold each I2C byte to force bit shift behavior 
    for(int i = COUNT_LEN - 1; i >= 0; i--){
      // Data[i] = Wire.read(); //Read im each data byte
      Val = Wire.read();
      Data = Data | (Val << (i)*8); //FIX!
    }

    return Data; 
  }
  else return 0; //Return empty value if error

}

bool PAC1934::setFrequency(Frequency SampleRate)
{ 
  uint8_t error = 0;
  uint8_t Temp = readByte(CTRL_REG, ADR, error); //Grab current value from the control register 
  if(error == 0) { //If no error on read, manipulate bits and write back
    Temp = Temp & 0x3F; //Clear sample bits
    Temp = Temp | (SampleRate << 6); //Set new sample rate
    writeByte(CTRL_REG, Temp, ADR); //Write updated value back
    update(); //Call non-destructive update to enforce new settings //FIX! Keep?
    delay(1); //Wait for new values 
    return true; //Return passing value
  }
  else return false; //Return fail if read error
}

//Check is overflow has occoured before last reset
bool PAC1934::testOverflow()
{
  uint8_t dummy;
  uint8_t Control = readByte(CTRL_REG, ADR, dummy); //Grab control reg
  if(Control & 0x01) return true;
  else return false;
}

float PAC1934::getPowerAvg(int Unit)
{
  bool dummy;
  return getPowerAvg(Unit, dummy);
}

float PAC1934::getPowerAvg(int Unit, bool &Stat)
{
  update();
  delay(1);
  bool CurrentDir = getCurrentDirection(Unit);
  uint8_t errorA = 0;
  uint32_t NumPoints = readCount(errorA); //Grab the number of points taken
  uint8_t errorB = 0;
  int64_t Val = readAccBlock(Unit, ADR, errorB); //Grab the desired accumulator block
  if(errorA == 0 && errorB == 0) { //If both errors are clean, proceed
    Stat = false; //Clear error flag
    float ValAvg = float(Val)/float(NumPoints); //Normalize for quantity acumulated //FIX! Check for overflow??
    float FSR = (3200.0/R[Unit]); //Find power FSR based on resistance of given sense resistor
    float P_Prop = 0; //Proportional current value, not yet scaled by CSR value  
    if(CurrentDir) P_Prop = float(ValAvg)/(134217728.0); //Divide by 2^27 to normalize 
    else P_Prop = float(ValAvg)/(268435456.0); //Divide by 2^28 to normalize 
    float PowerAvg = P_Prop*FSR; //Calculate average power 
    // Serial.println(NumPoints); //DEBUG!
    // Serial.println(float(Val)); //DEBUG!
    // Serial.println(ValAvg); //DEBUG! 
    // Print64(Val); //DEBUG!
    return PowerAvg; 
  }
  else {
    Stat = true; //Set error flag
    return 0;
  }
  

}

uint16_t PAC1934::readWord(uint8_t Reg, uint8_t Adr, uint8_t &Err)  //Send command value, returns entire 16 bit word
{
  Wire.beginTransmission(Adr);
  Wire.write(Reg);
  Err = Wire.endTransmission(); //Store Error

  if(Err == 0) { //Only proceed if ther is no error reported 
    unsigned long LocalTime = millis();
    Wire.requestFrom(Adr, 2, true);
    while(Wire.available() < 2 && (millis() - LocalTime) < I2C_Timeout);  
    uint8_t ByteHigh = Wire.read();  //Read in high and low bytes (big endian)
    uint8_t ByteLow = Wire.read();

    return ((ByteHigh << 8) | ByteLow); //DEBUG!
  }
  else return 0;
  
}

uint8_t PAC1934::readByte(uint8_t Reg, uint8_t Adr, uint8_t &Err)  //Send command value, returns byte
{
  Wire.beginTransmission(Adr);
  Wire.write(Reg);
  Err = Wire.endTransmission(); //Store Error

  
  if(Err == 0) { //Only proceed if ther is no error reported 
    unsigned long LocalTime = millis(); 
    Wire.requestFrom(Adr, 1, true);
    while(Wire.available() < 1 && (millis() - LocalTime) < I2C_Timeout); 
    return Wire.read();  //Read single byte
  }
  else return 0;
}

int64_t PAC1934::readAccBlock(uint8_t Unit, uint8_t Adr, uint8_t &Err)  //Read 48 bit accumulator values of the desired channel 
{
  Wire.beginTransmission(Adr);
  Wire.write(ACCUMULATOR_REG_0 + Unit); //Point to accumulator block plus offset
  Err = Wire.endTransmission(); //Store Error

  // uint8_t Data[6] = {0}; //Instatiate data array to store block in
  if(Err == 0) { //Only proceed if no error
    unsigned long LocalTime = millis();
    int64_t Data = 0; //Concatonated data
    Wire.requestFrom(Adr, BLOCK_LEN, true);
    while(Wire.available() < BLOCK_LEN && (millis() - LocalTime) < I2C_Timeout);  
    // return Wire.read();  //Read single byte
    uint64_t Val = 0; //Used to hold I2C reads
    for(int i = BLOCK_LEN - 1; i >= 0; i--) { //Drop entire block into Data array
      Val = Wire.read();
      Data = Data | uint64_t(Val << (i)*8); //Shift and accumulate each byte
      // Serial.println((BLOCK_LEN - i - 1)*8); //DEBUG!
      // Serial.println((BLOCK_LEN - i)*8); //DEBUG!
      // Print64(Data); //DEBUG!
      // Serial.println(Val, HEX); //DEBUG!
    }

    bool IsSigned = false; //Used to test of result should be signed, defaults to false 
    IsSigned = getVoltageDirection(Unit) | getCurrentDirection(Unit); //If either bus or sense is set to bi-directional, use signed math
    if((Data & 0x0000800000000000) && IsSigned) Data = Data | 0xFFFF000000000000; //Perform manual sign extension ONLY if using a signed value 

    //FIX! Check for errors before return

    // uint8_t ByteLow = Wire.read();

    // return ((ByteHigh << 8) | ByteLow); //DEBUG!
    return Data; //Return concatonated value
  }
  else return 0;
}

uint8_t PAC1934::writeByte(uint8_t Reg, uint8_t Data, uint8_t Adr)
{
  Wire.beginTransmission(Adr);
  Wire.write(Reg);
  Wire.write(Data);
  return Wire.endTransmission();
}

uint16_t PAC1934::getVoltageRaw(uint8_t Reg, bool Avg, uint8_t &Err)
{
  update(); //DEBUG!
  delay(1); //DEBUG!
  uint8_t Offset = 0; //Offset used to select for average
  if(Avg) Offset = 0x08; //Offset for average registers 
  uint8_t error = 0;
  uint16_t val = readWord(Offset + Reg, ADR, error);
  Err = error; //Pass error up
  if(Err == 0) return val;
  else return 0; //Return 0 if any error was indicated
}

// float PAC1934::GetBusVoltage(uint8_t Unit)
// {
//   return float(GetBusVoltageRaw(Unit))*(0.488); //Return result in mV
// }

uint8_t PAC1934::update(uint8_t Clear)
{
  uint8_t Reg = 0x1F; //Default to refesh, no clear
  if(!Clear) Reg = 0x1F;
  else if(Clear) Reg = 0x00;

  // Wire.beginTransmission(ADR);
  // Wire.write(Reg);
  // return Wire.endTransmission();
  // Wire.write(0x00); //Initilize a clear

  Wire.beginTransmission(ADR);
  Wire.write(Reg);
  Wire.write(0x00); //Initilize a clear
  uint8_t Error = Wire.endTransmission();
  // delay(2); //FIX! Find exact delay required 
  return Error;
}

void PAC1934::print64(uint64_t Data) { //Print out 64 bit value in chunks 
  for(int i = 0; i < 8; i++) { 
    uint8_t SubValue = Data >> (8 - i - 1)*8;
    Serial.print(SubValue, HEX); 
  }
  Serial.print("\n");

}

bool PAC1934::enableChannel(uint8_t Unit, bool State) { //Enable or disable reading of a given channel
  uint8_t error = 0;
  uint8_t CurrentState = readByte(CHANNEL_REG, ADR, error);
  if(error == 0) { //If read is successful, adjust values and write
    CurrentState = CurrentState & (~(0x01 << (7 - Unit))); //Clear enable bit in question
    CurrentState = CurrentState | (!State << (7 - Unit)); //Apply desired state to channel 
    writeByte(CHANNEL_REG, CurrentState, ADR); //DEBUG! Turn off measurment of all channels but channel 1
    return true; //return passing value
  }
  else return false; //If error in reading, throw false
}

bool PAC1934::setAddress(uint8_t _ADR)
{
  if(_ADR >= 0x10 && _ADR <= 0x1F) {
    ADR = _ADR; //Check range and set
    return true; //Return success
  }
  else return false; //Return failure due to range
}









