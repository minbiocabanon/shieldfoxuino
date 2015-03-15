/*
	Arduino in transparent mode from hardware serial to software serial linked to TD1202

	Receives from the hardware serial, sends to software serial.
	Receives from software serial, sends to hardware serial.

	The circuit: 
	* RX is digital pin 3 (connect to TX of TD1202)
	* TX is digital pin 2 (connect to RX of TD1202)
 
 */
#include <SoftwareSerial.h>

SoftwareSerial TD1202Serial(3, 2); // RX, TX

void setup(){
	// Open serial communications and wait for port to open:
	Serial.begin(9600);
	Serial.println("Transparent mode for TD1202");
	Serial.println("TD1202 is 9600 bauds");
	// set the data rate for the SoftwareSerial port
	TD1202Serial.begin(9600);
}

void loop(){
	if (TD1202Serial.available())
		Serial.write(TD1202Serial.read());
	if (Serial.available())
		TD1202Serial.write(Serial.read());
}
