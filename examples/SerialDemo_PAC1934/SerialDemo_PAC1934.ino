#include <PAC1934.h>

PAC1934 CSA(100, 100, 10, 10); //Instatiate CSA with 100mOhm CSR on Channel 1, 50mOhm CSR on Channel 2, 10mOhm CSR on Channel 3, 10mOhm CSR on Channel 4 

const unsigned long Period = 5000; //Number of ms between readings 

void setup() {
	Serial.begin(115200); //Initialize serial for communication
	while(!Serial); //Wait for Serial to turn on
	CSA.begin(); //Initialze CSA
	CSA.SetCurrentDirection(CH1, BIDIRECTIONAL); //Set channel 1 to read bi-directional current 
	CSA.SetCurrentDirection(CH2, UNIDIRECTIONAL); //Set channel 2 to read uni-directional current 
	CSA.SetCurrentDirection(CH3, UNIDIRECTIONAL); //Set channel 3 to read uni-directional current
	CSA.SetCurrentDirection(CH4, UNIDIRECTIONAL); //Set channel 4 to read uni-directional current  
	Serial.println("Fight the Power!");

}

void loop()
{
	float VBus[4] = {0};
	float Current[4] = {0};
	for(int i = 0; i < 4; i++) {
		VBus[i] = CSA.GetBusVoltage(CH1 + i, true); //Grab (averged) bus voltage form selected channel [V]
		Current[i] = CSA.GetCurrent(CH1 + i, true); //Grab (averaged) current (calculated based on CSR values) from selected channel [mA]
		printVIFormat(VBus[i], Current[i], i);
	}
	Serial.print("\n\r"); //Move to newline after each tranmission
	delay(Period);
}

void printVIFormat(float Voltage, float Current, int Channel) 
{
	String Print = "Bus" + String(Channel) + " = " + String(Voltage) + "V\tCurrent" + String(Channel) + " = " + String(Current) + "mA\t\t"; //Format substring
	Serial.print(Print); //Print substring
}