#include <PAC1932.h>

PAC1932 CSA(100, 50); //Instatitae with defualt address and 100mOhm on port 1 and 50mOhm on port 2

void setup() {
	Serial.begin(115200);
	while(!Serial); //Wait for Serial to turn on
	CSA.begin();
	CSA.SetCurrentDirection(CH1, BIDIRECTIONAL); //Set channel 1 to read bi-directional current 
	Serial.println("Fight the Power!");

}

void loop()
{
	static int i = 0; //DEBUG!
	if(CSA.TestOverflow()) Serial.println("ROLLOVER!"); 
	CSA.Update(); //Call synchronous update 
	float P1_Avg = CSA.GetPowerAvg(CH1); 
	float Current1 = CSA.GetCurrent(CH1, true);
	float VBus1 = CSA.GetBusVoltage(CH1);
	Serial.print("Power1 = ");
	Serial.print(P1_Avg*1000.0);
	Serial.print("mW\t");
	Serial.print("Bus1 = ");
	Serial.print(VBus1);
	Serial.print("V\t");
	Serial.print("I1 = ");
	Serial.print(Current1);
	Serial.print("mA\n");
	// float VBus1 = CSA.GetBusVoltage(CH1);
	// float VBus2 = CSA.GetBusVoltage(CH2);
	// float Current1 = CSA.GetCurrent(CH1, 40);
	// float Current2 = CSA.GetCurrent(CH2, 100);

	// Serial.print("Bus1 = ");
	// Serial.print(VBus1);
	// Serial.print("V\t");
	// Serial.print("Bus2 = ");
	// Serial.print(VBus2);
	// Serial.print("V\t");
	// Serial.print("I1 = ");
	// Serial.print(Current1);
	// Serial.print("mA\t");
	// Serial.print("I2 = ");
	// Serial.print(Current2);
	// Serial.println("mA\t");
	if(i++ == 5) { //DEBUG!
		CSA.Update();
		i = 0; 
	}
	delay(1000);
}