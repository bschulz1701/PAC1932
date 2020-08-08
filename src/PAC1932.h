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

#ifndef PAC1932_h
#define PAC1932_h

#include "Arduino.h"

#define SENSE1 0x0B
#define SENSE2 0x0C
#define BUS1 0x07
#define BUS2 0x08

#define BIDIRECTIONAL 1
#define UNIDIRECTIONAL 0

#define NEG_PWR 0x1D 

#define CSA_ADR 0x10

#define COUNT_REG 0x02
#define BUS_REG 0x07

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
    CH2 = 1
};

class PAC1932
{
  public:

    PAC1932(int _ADR);
    PAC1932(void);

    bool begin(void);
    float GetBusVoltage(uint8_t Unit, bool Avg = false); //Do not take average by default 
    float GetSenseVoltage(int Unit, bool Avg = false); //Do no take average by default 
    float GetCurrent(int Unit, float R, bool Avg = false); //Do not take average by default 
    void SetVoltageDirection(uint8_t Unit, bool Direction); 
    void SetCurrentDirection(uint8_t Unit, bool Direction);
    bool GetVoltageDirection(uint8_t Unit);
    bool GetCurrentDirection(uint8_t Unit);

  private:
    int ADR;
    int GetConfig(Config Value);
    int SetConfig(Config Value, uint8_t NewVal);
    int64_t ReadBlock(uint8_t Unit);
    uint32_t ReadCount();
    uint16_t ReadWord(uint8_t Reg, uint8_t Adr);
    uint16_t GetVoltageRaw(uint8_t Unit, bool Avg = false); //Keep private?? FIX! //By default do not take average 
    uint8_t Update(uint8_t Clear = false); //Keep privarte?? FIX!
    uint8_t WriteByte(uint8_t Reg, uint8_t Data, uint8_t Adr);
    uint8_t ReadByte(uint8_t Reg, uint8_t Adr);
};

#endif