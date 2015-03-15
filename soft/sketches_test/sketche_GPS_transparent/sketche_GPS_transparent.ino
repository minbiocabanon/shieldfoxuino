/*
	Arduino in transparent mode from hardware serial to software serial linked to TD1202

	Receives from the hardware serial, sends to software serial.
	Receives from software serial, sends to hardware serial.

	The circuit: 
	* RX is digital pin 4 (connect to TX of TD1202)
	* TX is digital pin 5 (connect to RX of TD1202)
 
 */
#include <SoftwareSerial.h>

SoftwareSerial GPSSerial(4, 5); // RX, TX

void setup(){
	// Open serial communications and wait for port to open:
	Serial.begin(9600);
	Serial.println("Transparent mode for GPS");
	Serial.println("TD1202 is 4800 bauds");
	// set the data rate for the SoftwareSerial port
	GPSSerial.begin(9600);
}

void loop(){
	if (GPSSerial.available())
		Serial.write(GPSSerial.read());
	if (Serial.available())
		GPSSerial.write(Serial.read());
}
