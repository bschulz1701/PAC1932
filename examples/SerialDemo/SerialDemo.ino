#include <PAC1932.h>

PAC1932 CSA;

void setup() {
	Serial1.begin(115200);
	while(!Serial1); //Wait for Serial1 to turn on
	CSA.begin();
	CSA.SetCurrentDirection(CH1, BIDIRECTIONAL); //Set channel 1 to read bi-directional current 
	Serial1.println("Fight the Power!");

}

void loop()
{
	float VBus1 = CSA.GetBusVoltage(CH1);
	float VBus2 = CSA.GetBusVoltage(CH2);
	float Current1 = CSA.GetCurrent(CH1, 40);
	float Current2 = CSA.GetCurrent(CH2, 100);

	Serial1.print("Bus1 = ");
	Serial1.print(VBus1);
	Serial1.print("V\t");
	Serial1.print("Bus2 = ");
	Serial1.print(VBus2);
	Serial1.print("V\t");
	Serial1.print("I1 = ");
	Serial1.print(Current1);
	Serial1.print("mA\t");
	Serial1.print("I2 = ");
	Serial1.print(Current2);
	Serial1.println("mA\t");
	delay(1000);
}