//OneWire von: https://github.com/prashantdev/arduino-OneWire
//DallasTemperature von https://github.com/prashantdev/Arduino-Temperature-Control-Library
//Siehe: http://technobabble.prithvitech.com

#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <EEPROM.h>
#include <JeeLib.h>

#define Serialprint2(x,y) Serial.print(x,y)
#define Serialprintln2(x,y) Serial.println(x,y)
#define Serialprint(x) Serial.print(x)
#define Serialprintln(x) Serial.println(x)
#define Serialbegin(x) Serial.begin(x)
#define Serialflush() Serial.flush()

//global variables

//1-Wire Bus
#define TEMPERATURE_PRECISION 12
#define NR_BUSLINES 8
#define NR_SENSORS_PER_BUS 15

OneWire oW[NR_BUSLINES];
DallasTemperature tSensors[NR_BUSLINES]; // Pass our oneWire reference to Dallas Temperature. 
DeviceAddress Thermometer[NR_BUSLINES][NR_SENSORS_PER_BUS]; // arrays to hold device addresses, NrBus, NrThermo
byte NrOfSensorsFound[NR_BUSLINES];

//1-Wire Bus: function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (byte i = 7; i > 0; i--) //Eigentlich ab 8 loslaufen, doch den CRC (?) sparen wir uns.
  {
    // zero pad the address if necessary
    if (deviceAddress[i-1] < 16) Serialprint("0");
    Serialprint2(deviceAddress[i-1], HEX);
  }
}

//Feuchte Sensor
PortI2C myport (0, 9/*PortI2C::KHZ100 */);
DeviceI2C FeuchteSensor (myport, 0x28); //HYP Feuchte und Temp Sensor


//****************************************************************
// Init Routines
//****************************************************************

//One Wire Setup
void InitOneWireTempMeasurement(byte BusNr) {
	//1-Wire Init
  
	// Start up the library
	oW[BusNr].setPin(BusNr+2);
	tSensors[BusNr].setOneWire(&oW[BusNr]);
	tSensors[BusNr].begin();
	tSensors[BusNr].setWaitForConversion(0);

	// locate devices on the bus
	NrOfSensorsFound[BusNr] = tSensors[BusNr].getDeviceCount();
	Serialprint("Found ");
	Serialprint2(NrOfSensorsFound[BusNr], DEC);
	Serialprintln(" devices.");

	// report parasite power requirements
	Serialprint("Parasite power is: "); 
	if (tSensors[BusNr].isParasitePowerMode()) {
		Serialprintln("ON");
	} else {
		Serialprintln("OFF");
	}

	Serialprintln("Sensors:1Wire");
	Serialprintln("Bus\tDevice\tID\tResolution");
	for (byte sn=0; sn<NR_SENSORS_PER_BUS; sn++) {
		Serialprint(BusNr);
		Serialprint("\t");
		Serialprint(sn);
		Serialprint("\t");
		
		if (!tSensors[BusNr].getAddress(Thermometer[BusNr][sn], sn)) {
			Serialprintln("no sensor.");
		} else {

			// show the addresses we found on the bus
			printAddress(Thermometer[BusNr][sn]);

			// set the resolution
			tSensors[BusNr].setResolution(Thermometer[BusNr][sn], TEMPERATURE_PRECISION);
			Serialprint("\t");
			Serialprintln2(tSensors[BusNr].getResolution(Thermometer[BusNr][sn]), DEC);
		}
	}
}


//********************************************************************
//Measurement Routines
//********************************************************************
void DoOneWireTempMeasurementStep1(byte BusNr) {
  //1-Wire Bus
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  tSensors[BusNr].requestTemperatures();
}

//Between Step 1 and Step 2 necessary:
//  delay(750);
void DoOneWireTempMeasurementStep2(byte BusNr) {
	float tempC;
	for (byte sn=0; sn<NrOfSensorsFound[BusNr]; sn++) {
		tempC = tSensors[BusNr].getTempC(Thermometer[BusNr][sn]);
		Serialprint(BusNr);
		Serialprint("\t");
		Serialprint(sn);
		Serialprint("\t");
		printAddress(Thermometer[BusNr][sn]);
		Serialprint("\t");
		Serialprintln2(tempC,3);
	}
}
//********************************************************************






//****************************************************************
// Init Routines
//****************************************************************

//HYP Sensor
void InitHYPSensorMeasurement() {
/*  Serialprint("HYP Sensor: ");
  if (FeuchteSensor.isPresent()) {
    Serialprintln("vorhanden.");
  } else { 
    Serialprintln("nicht vorhanden.");
  }
  */
}  



//********************************************************************
//Measurement Routines
//********************************************************************
void DoHYPSensorMeasurementStep1() {
  FeuchteSensor.send();
  FeuchteSensor.write(0); //value to write = 0
  FeuchteSensor.stop();
}
//Between Step 1 and Step 2 necessary:
//  delay(100);
void DoHYPSensorMeasurementStep2() {
  byte result[4];
  
  FeuchteSensor.receive();
  result[0] = FeuchteSensor.read(0); //More Bytes to read
  result[1] = FeuchteSensor.read(0);
  result[2] = FeuchteSensor.read(0);
  result[3] = FeuchteSensor.read(1); //Last byte to read
  FeuchteSensor.stop();
  
  
  float Temp;
  Temp = (result[3] >> 2);
  Temp = ((result[2] * 64) + Temp);
  Temp = 165./16384. * Temp - 40.;
  
  float Cap;
  Cap = (result[0] & 0x3FFF) * 256 + result[1];
  Cap = 100 / 16384. * Cap;

  Serialprintln("Status\tTemp\tHum");
  Serialprint(result[0] >> 6);
  Serialprint("\t");
  Serialprint2(Temp,4);
  Serialprint("\t");
  Serialprint2(Cap,4);
  Serialprintln("");
}

//********************************************************************






//********************************************************************

void setup() {
	Serialbegin(57600);
	Serialprintln("\nReadout 1Wire Sensors");
 
	for (byte i=0; i<NR_BUSLINES; i++) {
		NrOfSensorsFound[i] = 0;
		InitOneWireTempMeasurement(i);
	}
}
//********************************************************************


void loop() {

	Serialprintln("Start");

	InitHYPSensorMeasurement();
	DoHYPSensorMeasurementStep1();

	Serialprintln("Sensors:1Wire");
	Serialprintln("Bus\tDevice\tID\tTemp");
	for (byte i=0; i<NR_BUSLINES; i++) {
		DoOneWireTempMeasurementStep1(i);
	}
	delay(750); //Falls nur HYT dann nur 100ms notwendig
	for (byte i=0; i<NR_BUSLINES; i++) {
		DoOneWireTempMeasurementStep2(i);
	}
	
	
	Serialprintln("Sensors:HYT");
	DoHYPSensorMeasurementStep2();
	
	
	Serialprintln("Completed");
  
	Serialflush();
	delay(1000);
}
