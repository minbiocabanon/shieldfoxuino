//--------------------------------------------------
//! \file		sketche_sigfoxuino.ino
//! \brief		SIGFOX GPS TRACKER
//! \brief		Use GPS for localisation.
//! \brief		TD1202 SIGFOX module

//! \brief		Receives from the hardware serial, sends to software serial.
//! \brief		Receives from software serial, sends to hardware serial.
//! \brief		The circuit:
//! \brief		* RX is digital pin 3 (connect to TX of TD1202)
//! \brief		* TX is digital pin 2 (connect to RX of TD1202)
//! \brief		GPS with normal use of a TinyGPS++ (TinyGPSPlus) object.
//! \brief		It requires the use of SoftwareSerial, and assumes that you have a
//! \brief		4800-baud serial GPS device hooked up on pins 4(rx) and 5(tx).
//! \date		2015-May
//! \author		minbiocabanon
//--------------------------------------------------

#include <EEPROM.h>
#include <Time.h>
#include <Timer.h>
#include <TimeAlarms.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Shieldfox.h>

#include "sketche_sigfoxuino.h"

//flags
boolean bFlagTimeoutgpsfix = false ;
boolean bFlagDailyFix = false ;
boolean bFlagGPSfix = false;
boolean bFlagTransmitting = false;
boolean bFlagFirstFix = true;
boolean bFlagMsgInterval = true;
unsigned int sm_state = SM_NIL;
unsigned int ledgreen = 16;                 // LED GPS connected to digital pin 16
unsigned int ledorange = 17;                // LED RF12 connected to digital pin 17
static const int GPSRXPin = 4, GPSTXPin = 5;
static const uint32_t GPSBaud = 9600;
int EventTimeOutGPSFix;
unsigned long distanceKmToMyLoc;

int addr_eeprom_msg24h = 00;				// EEPROM address where is stored nb of msg sent by 24h
int addr_eeprom_msg = 01;					// EEPROM address where is stored total nb of message sent

// Offset hours from gps time (UTC)
const int offset = 0;   // UTC time -> don't use any zone, time zone will be compute on the ground side.

unsigned char Data_ascii[25];

stTrame Data;
stTrame_byte Data_byte;
	
TinyGPSPlus gps;							// The TinyGPS++ object
Shieldfox_ shieldfox;						// The shieldfox object
SoftwareSerial GpsSerial(GPSRXPin, GPSTXPin);	// The serial connection to the GPS device
time_t prevDisplay = 0; 					// when the digital clock was displayed
Timer t;

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
void readGPS(void){
	GpsSerial.listen();
	//some debug message, can be commented
	printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
	printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
	printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
	printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
	printInt(gps.location.age(), gps.location.isValid(), 5);
	printDateTime(gps.date, gps.time);
	printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
	printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
	printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
	printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);
	// end of debug 
	
	//compute distance in meter betweem GPS fix and MyLoc
	distanceKmToMyLoc =
		(unsigned long)TinyGPSPlus::distanceBetween(
			gps.location.lat(),
			gps.location.lng(),
			MY_LAT, 
			MY_LON);
	printInt(distanceKmToMyLoc, gps.location.isValid(), 9);
	Serial.println();
}

//----------------------------------------------------------------------
//!\brief         Set clock with GPS time
//!\param[in]     -
//!\return        -
//---------------------------------------------------------------------- 
void SetClockWithGPS(void){
	if( gps.location.isValid() && gps.location.age() < 500 ) {
		//GPS fix is OK, we can update the clock !
		if( bFlagFirstFix == true){
			//update time
			update_datetime();
			bFlagFirstFix = false;
			Alarm.alarmRepeat(12,00,00,DailyAlarm); // every day
			//Alarm.timerRepeat(300, DailyAlarm);  	// !!! TEST !!!
			//Alarm.timerRepeat(3600, HourAlarm);   // timer for every 1 hour 
			// Send a message for the first GPS Fix 
			bFlagDailyFix = true;			
		}
	}
}

//----------------------------------------------------------------------
//!\brief         Check if GPS position is inside area
//!\param[in]     -
//!\return        -
//---------------------------------------------------------------------- 
void CheckGeofencing(void){
	if( gps.location.isValid() && gps.hdop.value() <= HDOP_FIX_OK ) {
		// stop timeout gpsfix
		 t.stop(EventTimeOutGPSFix);
		// reset timeout fix flag 
		bFlagTimeoutgpsfix = false;
		// set GPS fix flag
		bFlagGPSfix = true;
		// if position is outside the authorised area
		if( distanceKmToMyLoc >= RADIUS_GEOFENCING && bFlagMsgInterval == true){
			Serial.println("D: -> ALARME , position is outisde authorised area !!!!!!!!!!!");			
			// set flags
			bFlagMsgInterval = false;
			// start 10 minutes interval between messages
			Alarm.timerOnce(INTERVAL_SIGFOX, TenMinInterval); 
			// switch to TRANSMIT mode for sending a message
			sm_state = SM_TRANSMIT ;
			Serial.println("D: -> transmit mode");			
		}
		else{
			// switch to STANDBY mode because everything is OK
			sm_state = SM_STANDBY ;
		}
	}
	else{
		//GPS is searching satellites
		bFlagGPSfix = false;
	}
}

//----------------------------------------------------------------------
//!\brief         Check if a daily fix occurs and then transmit a message
//!\param[in]     -
//!\return        -
//---------------------------------------------------------------------- 
void CheckDailyFix(void){
	if( bFlagDailyFix == true ){	
		if( gps.location.isValid() && gps.hdop.value() <= HDOP_FIX_OK ) {
			// switch to TRANSMIT mode for sending a message
			sm_state = SM_TRANSMIT ;
			Serial.println("D: -> Daily Fix - GPS fix ok , go to SM_TRANSMIT");	
		}
		else{
			Serial.println("D: -> Daily Fix - GPS NOK , DO nothing ???");			
		}
		// reset flag because GPS is not OK
		bFlagDailyFix = false;
	}
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
int ConvertTrame(stTrame *Data, stTrame_byte *Data_byte) {
  Data_byte->lattitude   = (long)( Data->ln_lattitude * 1000000 );
  Data_byte->longitude   = (long)( Data->ln_longitude * 1000000 );
  Data_byte->cap         = (unsigned char)( Data->f_cap / 10 );
  Data_byte->vitesse     = (unsigned char)Data->f_vitesse;
  Data_byte->temperature = (char)Data->f_temperature;
  Data_byte->tension     = (unsigned char)( Data->f_tension * 10) ;
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
void hexDump (char *desc, void *addr, int len) {
    int i;
    unsigned char *pc = (unsigned char*)addr;

    // Output description if given.
    if (desc != NULL){
		Serial.print (desc);
		Serial.println(":");
	}
    // Process every byte in the data.
    for (i = 0; i < len; i++) {
		Serial.print(" ");
		Serial.print(pc[i], HEX);
    }
	Serial.println(" ");
}
//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
void hexascii (void *addr_buff_byte, void *addr_buff_ascii, int len) {
	unsigned char *pc = (unsigned char*)addr_buff_byte;
	unsigned char *buffTmp = (unsigned char*)addr_buff_ascii;
	
	byte j = 0;
	for(byte i=0; i <len ; i++){
		buffTmp[j] = ( pc[i] & 0xF0) >> 4;
		if (buffTmp[j] <= 0x09)
			buffTmp[j] = buffTmp[j] + 0x30;
		else	
			buffTmp[j] = buffTmp[j] + 0x41 - 0x0A;	
		j++;
		
		buffTmp[j] = pc[i] & 0x0F;
		if (buffTmp[j] <= 0x09)
			buffTmp[j] = buffTmp[j] + 0x30;
		else	
			buffTmp[j] = buffTmp[j] + 0x41 - 0x0A;
		j++;
	}
	buffTmp[j++] = '\0';
	
	// DEBUG 
	// Serial.print("buffer ascii :");
	// for (int i = 0; i < 25; i++) {
		// Serial.print(buffTmp[i]);
		// Serial.print(" ");
    // }
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
void update_datetime(void){
		Serial.print("D: -> update date/time : ");
		// set the Time to the latest GPS reading
		setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
		adjustTime(offset * SECS_PER_HOUR);
		// prevDisplay = now();
		digitalClockDisplay();  
	

	// if (timeStatus()!= timeNotSet) {
		// if (now() != prevDisplay) { //update the display only if the time has changed
		  // prevDisplay = now();
		  // digitalClockDisplay();  
		// }
	// }	
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
void transmitmessage(void){
	
	GpsSerial.end();
		
	digitalWrite(ledorange, HIGH);
	Serial.println("D: in transmitmessage() function");
	
	Data.ln_lattitude = gps.location.lat();
	Data.ln_longitude = gps.location.lng();
	Data.f_cap = (int) gps.course.deg();
	Data.f_vitesse = (int) gps.speed.kmph();
	Data.f_temperature = 25.0;
	Data.f_tension = 3.30;

	ConvertTrame(&Data, &Data_byte);	
	//DEBUG
	hexDump ("Data_byte", &Data_byte, sizeof (Data_byte));
	
	// hexascii(&Data_byte, &Data_ascii, sizeof (Data_byte));
	
	// Serial.print("buffer ascii :");
	// for (int i = 0; i < 25; i++) {
		// Serial.print(Data_ascii[i]);
		// Serial.print(" ");
    // }
	// Serial.println();
	
	Serial.print("D: Sending message over Sigfox");
	//send buffer on sigfox network
	if(shieldfox.send(&Data_byte, sizeof(Data_byte)) == true){
		// if sent successfull
		// read eeprom counter values
		Serial.println("Sigfox module OK (SFM msg)");
		Serial.println("D: Reading EEPROM values");
		int valuemsg24h = EEPROM.read(addr_eeprom_msg24h);
		int valuemsg = EEPROM.read(addr_eeprom_msg);
		Serial.print("D: value : 24h = ");
		Serial.print(valuemsg24h);
		Serial.print(", total = ");
		Serial.println(valuemsg);
		// incremente counter values
		EEPROM.write(addr_eeprom_msg24h, ++valuemsg24h);
		EEPROM.write(addr_eeprom_msg, ++valuemsg);
		Serial.print("D: incremented EEPROM value : 24h = ");
		Serial.print(valuemsg24h, DEC); 
		Serial.print(", total = ");
		Serial.println(valuemsg, DEC);
	}
	else{
		Serial.println("Sigfox module return NOK (SFM msg)");
	}
		
	// go back to STANDBY mode
	sm_state = SM_STANDBY;
	// reset daily flag to not send a position again
	bFlagDailyFix = false ;
	digitalWrite(ledorange, LOW);
	
	GpsSerial.begin(GPSBaud);
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
void timeoutgpsfix(){

	// If we are in the daily fix, we force a TRANSMIT mode
	if ( bFlagDailyFix == true){
		// reset flag
		bFlagDailyFix = false ;
		// GPS running since TIMEOUT_GPS_FIX without fix, flag set !
		bFlagTimeoutgpsfix = true;
		// switch to TRANSMIT mode for sending a message
		sm_state = SM_TRANSMIT ;	
	}
	else{
		// GPS running since TIMEOUT_GPS_FIX without fix, flag set !
		bFlagTimeoutgpsfix = true;
		//go back un STANDBY mode
		sm_state = SM_STANDBY;
	}
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
void greenledtask(){

	if(bFlagGPSfix == true){
		digitalWrite(ledgreen, HIGH);
		smartDelay(50);
		digitalWrite(ledgreen, LOW);
	}
	else{
		digitalWrite(ledorange, HIGH);
		smartDelay(50);
		digitalWrite(ledorange, LOW);
	}
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
	unsigned long start = millis();
	do{
		while (GpsSerial.available())
			gps.encode(GpsSerial.read());
	}while (millis() - start < ms);
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
// functions to be called when an alarm triggers:
void TenMinInterval(void){
	Serial.println("D: TenMinInterval expired !!!!!!!!!!!");
	//reset flag to allow sending sigfox message again
	bFlagMsgInterval = true;

}
//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
// functions to be called when an alarm triggers:
void DailyAlarm(void){
	Serial.println("D: PROCEED TO DAILY FIX !!!!!!!!!!!");
	bFlagDailyFix = true;
	
	// erase 24h msg counter stored in eeprom
	EEPROM.write(addr_eeprom_msg24h, 00);
	//launch timeout fix GPS 
	t.after(TIMEOUT_GPS_FIX, timeoutgpsfix);
	sm_state = SM_ACTIVE;
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
// functions to be called when an alarm triggers:
void HourAlarm(void){
	Serial.println("D: PROCEED TO HOUR FIX !!!!!!!!!!!");
	bFlagDailyFix = true;
	
	// erase 24h msg counter stored in eeprom
	EEPROM.write(addr_eeprom_msg24h, 00);
	//launch timeout fix GPS 
	t.after(TIMEOUT_GPS_FIX, timeoutgpsfix);
	sm_state = SM_ACTIVE;
}


//----------------------------------------------------------------------
//!\brief       set system into the sleep state   
//!\brief       system wakes up when wtchdog is timed out 
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
void system_sleep() {
	// Serial.println("D: sleep mode ZZzz");
	// smartDelay(500);
	// Serial.println("D: exit from sleep mode");
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
static void printInt(unsigned long val, bool valid, int len)
{
	char sz[32] = "*****************";
	if (valid)
		sprintf(sz, "%ld", val);
	sz[len] = 0;
	for (int i=strlen(sz); i<len; ++i)
		sz[i] = ' ';
	if (len > 0) 
		sz[len-1] = ' ';
	Serial.print(sz);
	smartDelay(0);
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
	if (!d.isValid()){
		Serial.print(F("********** "));
	}
	else{
		char sz[32];
		sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
		Serial.print(sz);
	}

	if (!t.isValid()){
		Serial.print(F("******** "));
	}
	else{
		char sz[32];
		sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
		Serial.print(sz);
	}

	printInt(d.age(), d.isValid(), 5);
	smartDelay(0);
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
static void printStr(const char *str, int len)
{
	int slen = strlen(str);
	for (int i=0; i<len; ++i)
		Serial.print(i<slen ? str[i] : ' ');
	smartDelay(0);
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
void digitalClockDisplay(){
	// digital clock display of the time
	Serial.print(hour());
	printDigits(minute());
	printDigits(second());
	Serial.print(" ");
	Serial.print(day());
	Serial.print(" ");
	Serial.print(month());
	Serial.print(" ");
	Serial.print(year()); 
	Serial.println(); 
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
void printDigits(int digits) {
	// utility function for digital clock display: prints preceding colon and leading 0
	Serial.print(":");
	if(digits < 10)
		Serial.print('0');
	Serial.print(digits);
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
void statemachine(void){
	// do action follow state machine status
	switch(sm_state) {
		case SM_NIL:
			// Do nothing
			break;
		case SM_STANDBY:
			system_sleep();
			// at startup, launch timeout fix GPS 
			t.after(TIMEOUT_GPS_FIX, timeoutgpsfix);
			//switch to ACTIVE mode
			sm_state = SM_ACTIVE;
			break;
		case SM_ACTIVE:
			// get GPS position
			readGPS();
			// compare GPS position with autorized area
			CheckGeofencing();
			// check if a daily fix and message has to be sent
			CheckDailyFix();
			// set clock with GPS time
			SetClockWithGPS();
			break;
		case SM_TRANSMIT:
			// send a message
			transmitmessage();
			break;			
	}
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
void setup(){
	// Open serial communications and wait for port to open:
	Serial.begin(115200);
	
	setTime(18,20,0,10,8,15); 					// set time to Saturday 8:29:00am Jan 1 2011
	// Alarm.alarmRepeat(21,37,0,DailyAlarm);	// 5:45pm every day

	GpsSerial.begin(GPSBaud);
	// Initialize sigfox modem
	shieldfox.begin();
	pinMode(ledgreen, OUTPUT);      // sets the digital pin as output
	pinMode(ledorange, OUTPUT);      // sets the digital pin as output
	digitalWrite(ledgreen, HIGH);
	digitalWrite(ledorange, LOW);
	
	Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card   Distance "));
	Serial.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----   to Myloc "));
  
	// at startup, launch timeout fix GPS 
	EventTimeOutGPSFix =  t.after(TIMEOUT_GPS_FIX, timeoutgpsfix);
	t.every(PERIOD_CHECK_FIX, greenledtask);
	
	sm_state = SM_STANDBY;
  	Serial.println("Setup done.");	
}

//----------------------------------------------------------------------
//!\brief         
//!\param[in]     
//!\return        
//---------------------------------------------------------------------- 
void loop(){
	statemachine();	
	//for shedulers
	t.update();
	Alarm.delay(0);
	smartDelay(PERIOD_CHECK_FIX);
}
