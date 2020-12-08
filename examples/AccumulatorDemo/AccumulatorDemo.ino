#include <PAC1932.h>

PAC1932 CSA(100, 50); //Instatitae with defualt address and 100mOhm on port 1 and 50mOhm on port 2
volatile bool OverflowEvent = false; //Set when overflow occours, set by ISR, cleared in loop

void setup() {
	Serial.begin(115200);
	while(!Serial); //Wait for Serial to turn on
	CSA.begin();
	CSA.SetFrequency(SPS_256);
	CSA.SetCurrentDirection(CH1, UNIDIRECTIONAL); //Set channel 1 to read bi-directional current 
	CSA.Update(true);
	pinMode(2, INPUT_PULLUP); //DEBUG!
	attachInterrupt(digitalPinToInterrupt(2), Overflow, FALLING); //FIX! Make conditional/seperate in demo?
	Serial.println("Fight the Power!");

}

void loop()
{
	static int i = 0; //DEBUG!
	// if(CSA.TestOverflow()) Serial.println("ROLLOVER!"); 
	// if(OverflowEvent) {
	if(CSA.TestOverflow()) { //Check via software if interrupt has occoured 
		Serial.println("ROLLOVER!"); //Report event
		Serial.print("Regs-Pre:\t");
		Serial.print(CSA.ReadByte(0x01), HEX);
		Serial.print("\t");
		Serial.println(CSA.ReadByte(0x15), HEX);
		CSA.Update(true); //Call synchronous update, clear accumulator on overflow 
		Serial.print("Regs-Post:\t");
		Serial.print(CSA.ReadByte(0x01), HEX);
		Serial.print("\t");
		Serial.println(CSA.ReadByte(0x15), HEX);
		OverflowEvent = false; //Clear flag
	} 
	else CSA.Update(); //Call synchronous update 
	float P1_Avg = CSA.GetPowerAvg(CH1); 
	float Current1 = CSA.GetCurrent(CH1, true);
	float VBus1 = CSA.GetBusVoltage(CH1);
	Serial.print("Power1 = ");
	Serial.print(P1_Avg*1000000.0);
	Serial.print("uW\t");
	Serial.print("Bus1 = ");
	Serial.print(VBus1);
	Serial.print("V\t");
	Serial.print("I1 = ");
	Serial.print(Current1);
	Serial.print("mA\t");
	Serial.println(millis()); //Print millis for debugging time passage //DEBUG! 
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
	// if(i++ == 5) { //DEBUG!
	// 	CSA.Update();
	// 	i = 0; 
	// }
	delay(10000);
}

void Overflow()
{
	OverflowEvent = true; //Set flag
}