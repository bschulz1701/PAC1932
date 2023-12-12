#include <PAC1932.h>

PAC1932 CSA(100, 50); //Instatiate CSA with 100mOhm CSR on Channel 1, and a 50mOhm CSR on Channel 2 

void setup() {
	Serial.begin(115200); //Initialize serial for communication
	while(!Serial); //Wait for Serial to turn on
	CSA.begin(); //Initialze CSA
	CSA.SetCurrentDirection(CH1, BIDIRECTIONAL); //Set channel 1 to read bi-directional current 
	SA.SetCurrentDirection(CH2, UNIDIRECTIONAL); //Set channel 1 to read uni-directional current 
	Serial.println("Fight the Power!");

}

void loop()
{
	float VBus1 = CSA.GetBusVoltage(CH1); //Grab bus voltage from Channel 1 [V]
	float VBus2 = CSA.GetBusVoltage(CH2); //Grab bus voltage from Channel 2 [V]
	float Current1 = CSA.GetCurrent(CH1); //Grab current (calculated based on CSR values) from Channel 1 [mA]
	float Current2 = CSA.GetCurrent(CH2); //Grab current (calculated based on CSR values) from Channel 2 [mA]

	Serial.print("Bus1 = "); //Print Bus1 value
	Serial.print(VBus1);
	Serial.print("V\t");
	Serial.print("Bus2 = "); //Print Bus2 value
	Serial.print(VBus2);
	Serial.print("V\t");
	Serial.print("I1 = "); //Print Current1 value
	Serial.print(Current1);
	Serial.print("mA\t");
	Serial.print("I2 = "); //Print Current2 value
	Serial.print(Current2);
	Serial.println("mA\t");
	delay(1000);
}