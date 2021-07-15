#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "wifi.h"

#define LED_TIMER 500

// servo
#include <Servo.h>
Servo servo1;

int servo1Angle = 90;
int servo1PotPin = 0;
int servo1PotRead;

// bridge-h
int ENA = 15; // D8
int IN1 = 14; // D5
int IN2 = 13; // D7
#define PWMRANGE 256
#define PWMFREQ 1000
int motorState = 0;
int motorSpeed = 0;

// quadrature encoder
#define ENCODER_LEFT_FUNCTIONA encoderLeftCounterA
#define ENCODER_LEFT_FUNCTIONB encoderLeftCounterB
#define ENCODER_LEFT_PINA 5 //D1
#define ENCODER_LEFT_PINB 4 //D2
#define ENCODER_LEFT_SIGNAL CHANGE
byte encoderLeftPinLast; // control
volatile signed int encoderLeftPulses; // the number of pulses
boolean encoderLeftDirection; // the rotation direction

volatile long encoderLeftPulsesSpeedPID; // the number of pulses for PID
volatile long encoderLeftPulsesSteeringPID; // the number of pulses for PID, must be reset at same time

// bof:ROS
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>


// Set the rosserial socket server IP address
IPAddress server(192,168,1,109);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;

// Make a chatter publisher
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
// Be polite and say hello
char hello[13] = "hello world!";

// encoder pulses publisher
std_msgs::Int32 encoderLeftPulsesPub;
ros::Publisher encoderPublisher("wheel", &encoderLeftPulsesPub);
#define ENCODER_PUB_TIMER 500

void encoderPub() {
	static long unsigned int encoderPubTimer = 0;
	if(millis() >= encoderPubTimer) {
		encoderPubTimer = millis() + ENCODER_PUB_TIMER;
		encoderLeftPulsesPub.data = encoderLeftPulses;
		encoderPublisher.publish( &encoderLeftPulsesPub );
	}
}

float steering_angle = 0;
float velocity_target = 0;

void twistMsgCb(const geometry_msgs::Twist& msg) {
	velocity_target = msg.linear.x;
	if(velocity_target == 0 || msg.angular.z == 0) {
		steering_angle = 0;
	} else {
		float radius = velocity_target / msg.angular.z;
		steering_angle = atan(0.15 / radius);
	}
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSubscribe("cmd_vel", &twistMsgCb);

// eof:ROS


IRAM_ATTR void encoderLeftCounterA() {
  // look for a low-to-high on channel A
  if (digitalRead(ENCODER_LEFT_PINA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_LEFT_PINB) == LOW) {
      encoderLeftPulses = encoderLeftPulses - 1;         // CW
      // pid
      encoderLeftPulsesSpeedPID--;
      encoderLeftPulsesSteeringPID--;
    } else {
      encoderLeftPulses = encoderLeftPulses + 1;         // CCW
      // pid
      encoderLeftPulsesSpeedPID++;
      encoderLeftPulsesSteeringPID++;
    }
  } else {
    // its low-to-high-to-low on channel A
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_LEFT_PINB) == HIGH) {
      encoderLeftPulses = encoderLeftPulses - 1;          // CW
      // pid
      encoderLeftPulsesSpeedPID--;
      encoderLeftPulsesSteeringPID--;
    } else {
      encoderLeftPulses = encoderLeftPulses + 1;          // CCW
      // pid
      encoderLeftPulsesSpeedPID++;
      encoderLeftPulsesSteeringPID++;
    }
  }
}

IRAM_ATTR void encoderLeftCounterB() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_LEFT_PINB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER_LEFT_PINA) == HIGH) {
      encoderLeftPulses = encoderLeftPulses - 1;         // CW
      encoderLeftPulsesSpeedPID--;
      encoderLeftPulsesSteeringPID--;
    }
    else {
      encoderLeftPulses = encoderLeftPulses + 1;         // CCW
      encoderLeftPulsesSpeedPID++;
      encoderLeftPulsesSteeringPID++;
    }
  }

  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_LEFT_PINA) == LOW) {
      encoderLeftPulses = encoderLeftPulses - 1;          // CW
      encoderLeftPulsesSpeedPID--;
      encoderLeftPulsesSteeringPID--;
    }
    else {
      encoderLeftPulses = encoderLeftPulses + 1;          // CCW
      encoderLeftPulsesSpeedPID++;
      encoderLeftPulsesSteeringPID++;
    }
  }
}

void motorGoFront(int speed) {
	if(motorState != 1) {
		digitalWrite(IN1, LOW);
		digitalWrite(IN2, HIGH);
		motorState = 1;
	}
	analogWrite(ENA, speed);
}

void motorGoBack(int speed) {
	if(motorState != -1) {
		digitalWrite(IN1, HIGH);
		digitalWrite(IN2, LOW);
		motorState = -1;
	}
	analogWrite(ENA, speed);
}

void motorStop() {
	if(motorState != 0) {
		analogWrite(ENA, 0);
		digitalWrite(IN1, LOW);
		digitalWrite(IN2, LOW);
		motorState = 0;
	}
}

void blinkLED() {
	static boolean ledstate = 0;
	static long unsigned int ledTimer = 0;
	if(millis() >= ledTimer) {
		ledTimer = millis() + LED_TIMER;
		ledstate = !ledstate;
		digitalWrite(LED_BUILTIN, ledstate);
	}
}

void testTwo() {
	
	// turn on motors
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);

	// accelerate from zero to maximum speed
	for (int i = 0; i < PWMRANGE; i++)
	{
		analogWrite(ENA, i);
		delay(50);
    	Serial.println(i);
	}

	// decelerate from maximum speed to zero
	for (int i = (PWMRANGE-1); i >= 0; --i)
	{
		analogWrite(ENA, i);
		delay(50);
    	Serial.println(i);
	}
	// now turn off motors
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
  	delay(1000);


	// turn on motors
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);

	// accelerate from zero to maximum speed
	for (int i = 0; i < PWMRANGE; i++)
	{
		analogWrite(ENA, i);
		delay(50);
    	Serial.println(i);
	}


	// decelerate from maximum speed to zero
	for (int i = (PWMRANGE-1); i >= 0; --i)
	{
		analogWrite(ENA, i);
		delay(50);
    	Serial.println(i);
	}
	// now turn off motors
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
  	delay(1000);

}

void setup() {

	pinMode(LED_BUILTIN, OUTPUT);
	
	pinMode(ENA, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	analogWriteRange(PWMRANGE);
	analogWriteFreq(PWMFREQ);

	Serial.begin(115200);
	Serial.println("Booting");

	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);

	while (WiFi.waitForConnectResult() != WL_CONNECTED) {
		Serial.println("Connection Failed! Rebooting...");
		delay(5000);
		ESP.restart();
	}

	// Port defaults to 8266
	// ArduinoOTA.setPort(8266);
	// Hostname defaults to esp8266-[ChipID]
	// ArduinoOTA.setHostname("myesp8266");
	// No authentication by default
	// ArduinoOTA.setPassword("admin");
	// Password can be set with it's md5 value as well
	// MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

	ArduinoOTA.onStart([]() {

		String type;

		if (ArduinoOTA.getCommand() == U_FLASH) {
			type = "sketch";
		} else {
			// U_FS
			type = "filesystem";
		}

		// NOTE: if updating FS this would be the place to unmount FS using FS.end()
		Serial.println("Start updating " + type);
	});

	ArduinoOTA.onEnd([]() {
		Serial.println("\nEnd");
	});

	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
	});

	ArduinoOTA.onError([](ota_error_t error) {

		Serial.printf("Error[%u]: ", error);

		if (error == OTA_AUTH_ERROR) {
			Serial.println("Auth Failed");
		} else if (error == OTA_BEGIN_ERROR) {
			Serial.println("Begin Failed");
		} else if (error == OTA_CONNECT_ERROR) {
			Serial.println("Connect Failed");
		} else if (error == OTA_RECEIVE_ERROR) {
			Serial.println("Receive Failed");
		} else if (error == OTA_END_ERROR) {
			Serial.println("End Failed");
		} });

	ArduinoOTA.begin();

	Serial.println("Ready");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	// servo
	servo1.attach(12); //D6
	servo1.write(90);

	// encoders
	encoderLeftDirection = false; 
	pinMode(ENCODER_LEFT_PINA,INPUT);
	pinMode(ENCODER_LEFT_PINB,INPUT);
	attachInterrupt(ENCODER_LEFT_PINA, ENCODER_LEFT_FUNCTIONA, ENCODER_LEFT_SIGNAL);
	attachInterrupt(ENCODER_LEFT_PINB, ENCODER_LEFT_FUNCTIONB, ENCODER_LEFT_SIGNAL);

	// bof:ROS
	// Set the connection to rosserial socket server
	nh.getHardware()->setConnection(server, serverPort);
	nh.initNode();

	// Another way to get IP
	Serial.print("IP = ");
	Serial.println(nh.getHardware()->getLocalIP());

	nh.subscribe(cmdVelSubscribe);
	nh.advertise(encoderPublisher);
	// Start to be polite
	nh.advertise(chatter);
	// eof:ROS

}

void loop() {
	ArduinoOTA.handle();

	servo1PotRead = analogRead(servo1PotPin); // reads the value of the potentiometer (value between 0 and 1023)
	servo1Angle = map(servo1PotRead, 0, 1023, 35, 145);  // scale it to use it with the servo (value between 0 and 180)

	servo1.write(servo1Angle);	
	delay(50);

	motorSpeed = map(servo1PotRead, 0, 1023, -200, 200);  // scale it to use it with the brige-h pwm

	Serial.print(steering_angle);
	Serial.print("\t");
	Serial.print(velocity_target);
	Serial.print("\t");
	Serial.print(encoderLeftPulses);
	Serial.print("\t");
	Serial.print(motorSpeed);
	Serial.print("\t");
	Serial.print(motorState);
	Serial.print("\t");
	Serial.print(servo1PotRead);
	Serial.print("\t");
	Serial.println(servo1Angle);

	if(motorSpeed > 10) {
		motorGoFront(abs(motorSpeed));
	} else if(motorSpeed < -10) {
		motorGoBack(abs(motorSpeed));
	} else {
		motorStop();
	}

	//testTwo();
	
	blinkLED();

	encoderPub();
	//nh.spinOnce();

/*
	// bof:ROS
	if (nh.connected()) {
		Serial.println("Connected");
		// Say hello
		str_msg.data = hello;
		chatter.publish( &str_msg );
	} else {
		Serial.println("Not Connected");
	}
	nh.spinOnce();
	// Loop exproximativly at 1Hz
	//delay(1000);
	delay(100);
	// eof:ROS
*/
	
}