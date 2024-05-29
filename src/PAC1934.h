/******************************************************************************
MCP3421.h
Library for MCP3421 ADC
Bobby Schulz @ Northern Widget LLC & UMN
2/20/2018

MCP3421 is a 18 bit ADC, PGA, and various modes of resolution operation in order to facilitate various data rates

"Simplicity is prerquisite for reliability"
-Edsger W. Dijkstra

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef PAC1934_h
#define PAC1934_h

#include "Arduino.h"

#define SENSE1 0x0B
#define SENSE2 0x0C
#define SENSE3 0x0D
#define SENSE4 0x0E
#define BUS1 0x07
#define BUS2 0x08
#define BUS3 0x09
#define BUS4 0x0A

#define BIDIRECTIONAL 1
#define UNIDIRECTIONAL 0

#define NEG_PWR 0x1D 

#define CSA_ADR 0x10

#define CTRL_REG 0x01
#define COUNT_REG 0x02
#define BUS_REG 0x07
#define CHANNEL_REG 0x1C
#define PRODUCT_ID_REG 0xFD

#define ACCUMULATOR_REG_0  0x03 //Begining of accumator blocks
#define BLOCK_LEN 6 //Length of accumulator block in bytes 
#define COUNT_LEN 3 //Number of bytes used for count value 

enum Config {  //Setup configuration values, upper byte is register address, lower byte is shift and mask, lower nibble is shift, upper nibble is mask
  C1RS = 0x0A32,
  C1RA = 0x0A30,
  C2RS = 0x0A36,
  C2RA = 0x0A34,
  C1CSS = 0x0B74, 
  C1SA = 0x0B32,
  C1SR = 0x0B30,
  C2CSS = 0x0C74,
  C2SA = 0x0C32,
  C2SR = 0x0C30
};

enum Channel {
    CH1 = 0,
    CH2 = 1,
    CH3 = 2,
    CH4 = 3,
};

enum Frequency {
  SPS_8 = 0b11,
  SPS_64 = 0b10,
  SPS_256 = 0b01,
  SPS_1024 = 0b00
};

class PAC1934
{
  public:

    PAC1934(float _R1 = 0, float _R2 = 0, float _R3 = 0, float _R4 = 0, int _ADR = CSA_ADR); //Default resistor values to 0 to disable certian calcs
    // PAC1934(void);

    bool begin(void);
    float getBusVoltage(uint8_t Unit, bool Avg, bool &Stat); //Do not take average by default, optional pass/fail return
    float getSenseVoltage(int Unit, bool Avg, bool &Stat); //Do no take average by default, optional pass/fail return
    float getCurrent(int Unit, bool Avg, bool &Stat); //Do not take average by default, optional pass/fail return
    float getBusVoltage(uint8_t Unit, bool Avg = false); //Do not take average by default 
    float getSenseVoltage(int Unit, bool Avg = false); //Do no take average by default 
    float getCurrent(int Unit, bool Avg = false); //Do not take average by default 
    void setVoltageDirection(uint8_t Unit, bool Direction); 
    void setCurrentDirection(uint8_t Unit, bool Direction);
    bool getVoltageDirection(uint8_t Unit);
    bool getCurrentDirection(uint8_t Unit);
    bool setFrequency(Frequency SampleRate);
    int getFrequency(); 
    float getPowerAvg(int Unit, bool &Stat);
    float getPowerAvg(int Unit);
    uint8_t update(uint8_t Clear = false); //Keep privarte?? FIX!
    bool testOverflow(); 
    bool enableChannel(uint8_t Unit, bool State = true);
    bool setAddress(uint8_t _ADR);

  private:
    int ADR;
    float R[4] = {0}; //CSR resistor values [mOhms]
    // bool DirV[2] = {false}; //Default to uni-directional
    // bool DirI[2] = {false}; //Default to uni-directional
    const unsigned long I2C_Timeout = 10; //Wait up to 10ms for device to respond 
    int getConfig(Config Value);
    int setConfig(Config Value, uint8_t NewVal);
    int64_t readBlock(uint8_t Unit, uint8_t &Err);
    uint32_t readCount(uint8_t &Err);
    uint16_t readWord(uint8_t Reg, uint8_t Adr, uint8_t &Err);
    uint16_t getVoltageRaw(uint8_t Unit, bool Avg, uint8_t &Err); //Keep private?? FIX! //By default do not take average 
    
    uint8_t writeByte(uint8_t Reg, uint8_t Data, uint8_t Adr);
    uint8_t readByte(uint8_t Reg, uint8_t Adr, uint8_t &Err); //DEBUG! Move back to private after testing
    int64_t readAccBlock(uint8_t Unit, uint8_t Adr, uint8_t &Err);
    void print64(uint64_t Data);//DEBUG!!!!
};

#endif