//--------------------------------------------------
//! \file		sketche_sigfoxuino.h
//! \brief		header file for structures and enumerate
//! \date		2015-Mar
//! \author		minbiocabanon
//--------------------------------------------------

#define		TIMEOUT_GPS_FIX		5*60*1000	// 5 minutes = 300 000 msec	
#define		PERIOD_CHECK_FIX	5*1000		// Period between position check -> 5 minutes = 300 000 msec		
#define		INTERVAL_SIGFOX		10*60		// INTERVAL between two messages, in minutes 
#define 	HDOP_FIX_OK			300			// HDOP without unit
// State machine
#define 	SM_NIL 				0			// No action SM
#define 	SM_STANDBY 			1			// Standby SM
#define 	SM_ACTIVE 			2			// Active SM
#define 	SM_TRANSMIT 		3			// Transmit SM
#define 	RADIUS_GEOFENCING	200			// radius in meter, outside this radius -> ALARM !

static const double MY_LAT = 43.791358, MY_LON = 1.107715;
//static const double MY_LAT = 41.791358, MY_LON = 2.107715; !! TEST !!

typedef struct trame {
long         lattitude;
long         longitude;
unsigned char cap;
unsigned char vitesse;
char          temperature;
unsigned char tension;
} stTrame_byte;

typedef struct trame_byte {
float ln_lattitude;
float ln_longitude;
float f_cap;
float f_vitesse;
float f_temperature;
float f_tension;
} stTrame;


//----------------------------------------------------------------------
//!\Prototypes             
//----------------------------------------------------------------------
int ConvertTrame(stTrame *Data, stTrame_byte *Data_byte);
void statemachine(void);
void readGPS(void);
void hexDump (char *desc, void *addr, int len) ;
void hexascii (stTrame_byte *Data_byte, void *addr);
void update_datetime(void);
void transmitmessage(void);
void timeoutgpsfix();
void greenledtask();
