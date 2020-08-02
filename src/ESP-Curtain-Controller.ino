/**
 *	OlaizCloud Cortinas v0.1
 */

	/**
	 *	Configuration
	 */

		#define MOTOR_PIN1 D2
		#define MOTOR_PIN2 D3
		#define MOTOR_PIN3 D4
		#define MOTOR_PIN4 D5

		/** 
		 *	Possible modes: 
		 *		MODE_PHASE_SINGLE:	one phase at a time (4 steps)
		 *		MODE_PHASE_DUAL:	two phases at a time (4 steps, more strength)
		 *		MODE_PHASE_HALF:	half step (8 steps, more precise, slightly less strenth, uses one and two phases depending on the step)
		 */	
		#define MOTOR_MODE MODE_PHASE_HALF

	 	// long datatype, examples: -3, -3.5, +1.5
		#define TIME_ZONE	-3

		#define BLYNK_HOST	IPAddress(192, 168, 1, 2)
		#define BLYNK_TOKEN	"2RCiAcRe_x6FBqj0cP6lmO_cwtnXQnzV"

		// Blynk slider resolution (8 bits = 0 to 255, 10 bits = 0 to 1023)
		#define BLYNK_SLIDER_RESOLUTION	10

		//#define CURTAIN_GLOBAL_CONTROL_PIN1		V0
		#define CURTAIN_CONTROL_PIN V10

		// Uncomment to enable debugging. 
		#define DEBUG_ENABLED
	/**
	 *	End config
	 */

	#define MODE_PHASE_SINGLE	1
	#define MODE_PHASE_DUAL		2
	#define MODE_PHASE_HALF		3

	#if MOTOR_MODE == MODE_PHASE_SINGLE
		// TO DO!
	#elif MOTOR_MODE == MODE_PHASE_DUAL
		// TO DO!
	#elif MOTOR_MODE == MODE_PHASE_HALF
		#define MOTOR_STEPS	8
		const uint8_t motor_step_table[MOTOR_STEPS] =
		{
			B1000,
			B1100,
			B0100,
			B0110,
			B0010,
			B0011,
			B0001,
			B1001
		};
	#else
		#error you must define MOTOR_MODE as MODE_PHASE_SINGLE, MODE_PHASE_DUAL or MODE_PHASE_HALF
	#endif

	#ifdef DEBUG_ENABLED
		#define BLYNK_PRINT		Serial

		#define DEBUG_INIT			Serial.begin(115200)
		#define DEBUG(code)			code
		#define DEBUG_PRINT(text)	Serial.print(text)
		#define DEBUG_PRINTLN(line)	Serial.println(line)
	#else
		#define DEBUT_INIT
		#define DEBUG(code)
		#define DEBUG_PRINT(line)
		#define DEBUG_PRINTLN(line)
	#endif

	#include <ESP8266WiFi.h>

	#include <DNSServer.h>
	#include <ESP8266WebServer.h>
	#include <WiFiManager.h>

	#include <BlynkSimpleEsp8266.h>

	#include <WiFiUdp.h>
	#include <NTPClient.h>

	#include <EEPROM.h>

	WiFiUDP ntp_udp;
	NTPClient ntp(ntp_udp, "pool.ntp.org", TIME_ZONE * 60 * 60);

	// It might be a little bit overkill (uint64_t), but we have plenty of margin to deal with the whole stepper resolution
	uint32_t curtain_max = 0;

 	uint32_t curtain_position = 0;
	uint32_t curtain_going_to = 0;

	uint8_t motor_current_step = 0;

	void setup()
	{
		// Configure the motor pins
		pinMode(MOTOR_PIN1, OUTPUT);
		pinMode(MOTOR_PIN2, OUTPUT);
		pinMode(MOTOR_PIN3, OUTPUT);
		pinMode(MOTOR_PIN4, OUTPUT);

		// Macro for debugs (begin's the Serial port)
		DEBUG_INIT;

		// Setup WiFiManager (Yeap, this lib is awesome!)
		WiFiManager wifi_manager;
		wifi_manager.autoConnect();

		// Setup and connect Blynk
		Blynk.config(BLYNK_TOKEN, BLYNK_HOST);
		Blynk.connect(); // while(!Blynk.connect());?

		ntp.begin();

		// If there's a power outage, we'll keep track of where the curtain is. 
		EEPROM.begin(128);
		// Get the current position from EEPROM. 
		EEPROM.get(0, curtain_max);
		EEPROM.get(0 + sizeof(curtain_max), curtain_position);

		curtain_going_to = curtain_position;

		// TO TEST
		curtain_max = 30000;
	}

	void motor_write(uint8_t step_id)
	{
		if(step_id > (MOTOR_STEPS - 1))
		{
			DEBUG_PRINT("Error: motor_write called with step_id > ");
			DEBUG_PRINTLN((MOTOR_STEPS - 1));

			return;
		}

		// DEBUG INFO

		digitalWrite(MOTOR_PIN1, motor_step_table[step_id] & (1<<0));
		digitalWrite(MOTOR_PIN2, motor_step_table[step_id] & (1<<1));
		digitalWrite(MOTOR_PIN3, motor_step_table[step_id] & (1<<2));
		digitalWrite(MOTOR_PIN4, motor_step_table[step_id] & (1<<3));
	}

	void motor_turn(bool clockwise)
	{
		if(clockwise)
		{
			if(motor_current_step == (MOTOR_STEPS - 1))
				motor_current_step = 0;
			else
				motor_current_step++;
		}
		else
		{
			if(motor_current_step == 0)
				motor_current_step = (MOTOR_STEPS - 1);
			else
				motor_current_step--;
		}

		motor_write(motor_current_step);

		//DEBUG_PRINT("Motor in: ");
		//DEBUG_PRINT(curtain_position)
		delay(1);
	}

	void loop()
	{
		//ntp.update();
		Blynk.run();

		if(curtain_going_to < curtain_position)
		{
			motor_turn(false);
			curtain_position--;
		}
		else if (curtain_going_to > curtain_position)
		{
			motor_turn(true);
			curtain_position++;
		}
	}

	BLYNK_WRITE(CURTAIN_CONTROL_PIN)
	{
		curtain_going_to = (uint64_t) (param.asInt() * ((float) curtain_max / (pow(2, BLYNK_SLIDER_RESOLUTION) - 1)));

		// Yeap, I know, those casts...
		DEBUG_PRINT("Curtain: from ");
		DEBUG_PRINT((uint32_t) curtain_position);
		DEBUG_PRINT(" to ");
		DEBUG_PRINT((uint32_t) curtain_going_to);
		DEBUG_PRINT(" (slider at ");
		DEBUG_PRINT(param.asInt());
		DEBUG_PRINTLN(")");
	}

/**
 *	Things to do: 
 *	
 *	- Implement a proper debug lib (I think I've made something before - Look in old projects)
 *	- if wifi disconnected and curtain is still, ESP.restart();
 *	- Implement wifiManager for Blynk also: https://www.esp8266.com/viewtopic.php?p=85713
 *	- BlynkWM or EasyBlynk8266
 *	- Test different modes
 *	- proper motor speed control
 *	- is uint32_t enough? do some math ;)

/*
	int pole1[] ={0,0,0,0, 0,1,1,1, 0};//pole1, 8 step values
int pole2[] ={0,0,0,1, 1,1,0,0, 0};//pole2, 8 step values
int pole3[] ={0,1,1,1, 0,0,0,0, 0};//pole3, 8 step values
int pole4[] ={1,1,0,0, 0,0,0,1, 0};//pole4, 8 step values]*
 */