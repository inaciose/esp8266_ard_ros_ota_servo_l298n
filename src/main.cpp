#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "wifi.h"

#include "pid.h"

// servo
#include <Servo.h>
Servo servo1;
int servo1Angle = 90;
int servo1PotPin = 0;
int servo1PotRead;

#define LED_TIMER 500

// bridge-h
int ENA = 15; // D8
int IN1 = 14; // D5
int IN2 = 13; // D7
#define PWMRANGE 256
#define PWMFREQ 1000
int motorState = 0;
//int motorSpeed = 0;
signed int leftMotorPwmOut = 0;
//signed int leftMotorPwmOutCmd = 0;

// quadrature encoder
#define ENCODER_LEFT_FUNCTIONA encoderLeftCounterA
#define ENCODER_LEFT_FUNCTIONB encoderLeftCounterB
#define ENCODER_LEFT_PINA 5 //D1
#define ENCODER_LEFT_PINB 4 //D2
#define ENCODER_LEFT_SIGNAL CHANGE
byte encoderLeftPinLast; // control
volatile signed int encoderLeftPulses; // the number of pulses
boolean encoderLeftDirection; // the rotation direction

long encoderLeftPulsesTarget = 0;
long encoderRightPulsesTarget = 0;
bool encoderLeftPulsesOnTarget = false;
bool encoderRightPulsesOnTarget = false;
bool encoderPulsesTargetEnabled = false;
int encoderLeftPulsesTargetStopOffset = 0;
int encoderRightPulsesTargetStopOffset = 0;

long encoderLeftPulsesTargetStart = 0;
long encoderRightPulsesTargetStart = 0;

// pid
volatile long encoderLeftPulsesSpeedPID; // the number of pulses for PID
volatile long encoderLeftPulsesSteeringPID; // the number of pulses for PID, must be reset at same time

float wheelbase = 0.22; // meters
float wheelradius = 0.033; // meters

// pulses per rotation
int ppr = 2000;
// pulses per meter
int ppm = 0;
// meters per pulse
float mpp = 0;
// meters per wheel rotation
float mpr = 0;

// instantaneous speed
float speed = 0.0;
// speed on second
float speedOs = 0.0;

int speedLastPulses = 0;
int speedOsLastPulses = 0;

// bof:ROS
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

// Set the rosserial socket server IP address
IPAddress server(192,168,1,64);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;

// encoder pulses publisher
std_msgs::Int32 encoderLeftPulsesPub;
ros::Publisher encoderPublisher("wheel", &encoderLeftPulsesPub);
#define ENCODER_PUB_TIMER 500

// mcu ip publisher
std_msgs::String ip_msg;
ros::Publisher ipPublisher("mcu_ip", &ip_msg);
#define IP_PUB_TIMER 10000

void encoderPub() {
	static long unsigned int encoderPubTimer = 0;
	if(millis() >= encoderPubTimer) {
		encoderPubTimer = millis() + ENCODER_PUB_TIMER;
		encoderLeftPulsesPub.data = encoderLeftPulses;
		encoderPublisher.publish( &encoderLeftPulsesPub );
	}
}

void ipPub() {
	static long unsigned int ipPubTimer = 0;
	if(millis() >= ipPubTimer) {
		ipPubTimer = millis() + IP_PUB_TIMER;
		ip_msg.data = WiFi.localIP().toString().c_str();
		ipPublisher.publish( &ip_msg );
	}
}

float steering_angle = 0;
float velocity_target = 0;

float twist_angular = 0;
float twist_linear = 0;

long unsigned int secondTimer = 0;
long unsigned int loopInfoTimer = 0;

void twistMsgCb(const geometry_msgs::Twist& msg) {
	
	int encoderPulsesTarget = 0;

	velocity_target = msg.linear.x;
	twist_linear = msg.linear.x;
	twist_angular = msg.angular.z;

	if(velocity_target == 0 || msg.angular.z == 0) {
		steering_angle = 0;
	} else {
		float radius = velocity_target / msg.angular.z;
		steering_angle = atan(wheelbase / radius);
	}

	steering_angle = -steering_angle;

	// dx = (l + r) / 2
	// dr = (r - l) / w
	//float speed_wish_right = (cmd_vel.angle*WHEEL_DIST)/2 + cmd_vel.speed;
	//float speed_wish_left = velocity_target * 2 - speed_wish_right;

	leftSpeedPidSetPointTmp = velocity_target * 2 - ((msg.angular.z * wheelbase ) / 2 + velocity_target);

	/*
	Serial.print(leftSpeedPidSetPointTmp);
	Serial.print("\t");
	*/

	leftSpeedPidSetPointTmp = (leftSpeedPidSetPointTmp * ppm) / SPEED_PID_SAMPLE_FREQ;

	leftSpeedPidSetPointTmp = -leftSpeedPidSetPointTmp;

	/*
	Serial.print(leftSpeedPidSetPointTmp);
	Serial.println("\n");
	*/

	// pid
	//leftSpeedPidSetPointTmp = velocity_target;
	// set left motors direction and speed
	if(leftSpeedPidSetPointTmp > 0) {
		leftSpeedPidSetPointDirection = 1;
		leftSpeedPidSetPoint = round(leftSpeedPidSetPointTmp);
	} else if(leftSpeedPidSetPointTmp < 0) {
		leftSpeedPidSetPointDirection = -1;
		leftSpeedPidSetPoint = round(abs(leftSpeedPidSetPointTmp));
	} else {
		leftSpeedPidSetPointDirection = 0;
		leftSpeedPidSetPoint = round(leftSpeedPidSetPointTmp);
	}

	leftSpeedPid.SetMode(AUTOMATIC);
	//useSteeringPid = false;
	//encoderLeftPulsesSteeringPID = encoderRightPulsesSteeringPID = 0;

	// set the target for encoders if any
	if(encoderPulsesTarget) {
		// set left encoder target
		if(leftSpeedPidSetPointDirection >= 0) {
			//encoderLeftPulsesTarget = encoderLeftPulses + (float)(abs(encoderLeftPulsesTarget) / 1000.0 * pulses_per_m);
			encoderLeftPulsesTarget = encoderLeftPulses + abs(encoderPulsesTarget);    
		} else {
			//encoderLeftPulsesTarget = encoderLeftPulses - (float)(abs(encoderLeftPulsesTarget) / 1000.0 * pulses_per_m);
			encoderLeftPulsesTarget = encoderLeftPulses - abs(encoderPulsesTarget);
		}

		encoderLeftPulsesOnTarget = false;
		encoderPulsesTargetEnabled = true;
		// debug only
		encoderLeftPulsesTargetStart = encoderLeftPulses;
	} else {
		encoderLeftPulsesTarget = 0;
		encoderLeftPulsesOnTarget = false;
		encoderPulsesTargetEnabled = false;
		// debug only
		encoderLeftPulsesTargetStart = 0;
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

void bodyMotorsControl() {
  // set motor left direction & speed
  if(leftMotorPwmOut == 0) {
	motorStop();
  } else if(leftMotorPwmOut > 0) {
    motorGoFront(leftMotorPwmOut);
  } else {
    motorGoBack(abs(leftMotorPwmOut));  
  }
}

boolean leftSpeedPidCompute() {
  boolean pidResult;
  leftSpeedPidInput = abs(encoderLeftPulsesSpeedPID);
  pidResult = leftSpeedPid.Compute(); 
  if(pidResult) {
    encoderLeftPulsesSpeedPID = 0;
    leftSpeedPidInputLast = leftSpeedPidInput;
	speed = (encoderLeftPulses - speedLastPulses) * mpp * SPEED_PID_SAMPLE_FREQ;
	speedLastPulses = encoderLeftPulses;
  }
  return pidResult;
}

void update_PID() {
  // calculate speedPid
  leftSpeedPidResult = leftSpeedPidCompute();

  if(leftSpeedPidSetPoint) {
	// show something to debug
	
	/*
	Serial.print(leftSpeedPidSetPoint);
	Serial.print("\t");
	Serial.print(leftSpeedPidInputLast);
	Serial.println("\n");
	*/
/*
	Serial.print(steering_angle);
	Serial.print("\t");
	Serial.print(velocity_target);
	Serial.print("\t");
	Serial.print(speedOs);
	Serial.print("\t");
	Serial.print(speed);
	Serial.print("\t");
	
	Serial.print(leftSpeedPidSetPoint);
	Serial.print("\t");
	Serial.print(leftSpeedPidInputLast);
	Serial.print("\t");
	
	Serial.print(encoderLeftPulses);
	Serial.print("\t");
	Serial.print(leftMotorPwmOut);
	Serial.print("\t");
	Serial.print(motorState);
	Serial.print("\t");
	Serial.println(servo1Angle);
*/	
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

void setup() {

	Serial.begin(115200);
	Serial.println("Booting...");

	pinMode(LED_BUILTIN, OUTPUT);

	// motor settings	
	pinMode(ENA, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	analogWriteRange(PWMRANGE);
	analogWriteFreq(PWMFREQ);

	// stop motors
	analogWrite(ENA, 0);
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	motorState = 0;

	// encoder & odometry settings
	// meters per wheel rotation
	mpr = (2 * PI * wheelradius);
	// pulses per meter
	ppm = ppr / mpr;
	// meters per pulse
	mpp = 1.0 / ppm;

	Serial.print("wheelbase: ");
	Serial.println(wheelbase, 3);
	Serial.print("wheelradius: ");
	Serial.println(wheelradius, 3);

	Serial.print("pulses per rotation: ");
	Serial.println(ppr, 6);
	Serial.print("meters per rotation: ");
	Serial.println(mpr, 6);
	Serial.print("meters per pulse: ");
	Serial.println(mpp, 6);
	Serial.print("pulses per meter: ");
	Serial.println(ppm);

	// WiFi
	Serial.println("WiFi connecting...");
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
	servo1.write(90 - 16);

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
	Serial.print("ROS IP = ");
	Serial.println(nh.getHardware()->getLocalIP());

	nh.subscribe(cmdVelSubscribe);
	nh.advertise(encoderPublisher);
	nh.advertise(ipPublisher);

	ipPub();

	// eof:ROS

	// left speed PID
	//leftSpeedPidSetPoint = SPEED_PID_TARGET;
	leftSpeedPidSetPoint = 0;
	//leftSpeedPid.SetMode(AUTOMATIC);
	leftSpeedPid.SetMode(MANUAL);
	leftSpeedPid.SetSampleTime(SPEED_PID_SAMPLE_TIME); 
	leftSpeedPid.SetOutputLimits(LEFT_SPEED_PID_MIN_OUTPUT, LEFT_SPEED_PID_MAX_OUTPUT);

}

void loop() {
	ArduinoOTA.handle();

	servo1Angle = map(steering_angle*100, -90, 90, 30, 150);  // scale it to use it with the servo (value between 0 and 180)
	
	servo1Angle -= 16; // go straight compensation

	if(servo1Angle < 30) servo1Angle = 30;
	if(servo1Angle > 135) servo1Angle = 135;

	if(twist_linear == 0 || twist_linear > 0.05) {
		servo1.write(servo1Angle);
	}
	
	//delay(10);

	blinkLED();

	encoderPub();
	ipPub();

	nh.spinOnce();

	static double encoderLeftPulsesLast = 999;
	
	if(encoderLeftPulses != encoderLeftPulsesLast) {
		if(encoderLeftPulses != encoderLeftPulsesLast) encoderLeftPulsesLast = encoderLeftPulses;
	}

	// check encoder targets
	if(encoderPulsesTargetEnabled) {
		//
		// check left encoder target
		if(!encoderLeftPulsesOnTarget) {
		if(leftSpeedPidSetPointDirection >= 0) {
			//Serial.print("LP>: "); Serial.println(encoderLeftPulses);
			if(encoderLeftPulses >= encoderLeftPulsesTarget - encoderLeftPulsesTargetStopOffset) {
			encoderLeftPulsesOnTarget = true;  
			}
		} else {
			//Serial.print("LP<: "); Serial.println(encoderLeftPulses);
			if(encoderLeftPulses < encoderLeftPulsesTarget + encoderLeftPulsesTargetStopOffset) {
			encoderLeftPulsesOnTarget = true;
			}      
		}
		// left stop on encoder target
		if(encoderLeftPulsesOnTarget) {
			//Serial.println("L ON TARGET");
			leftMotorPwmOut = 0;
			leftSpeedPidSetPoint = 0;
			leftSpeedPidSetPointDirection = 0; 
		}
		}
		
		//
		// encoders on target
		if(encoderLeftPulsesOnTarget) {
		//Serial.println("BOTH ON TARGET");

		encoderLeftPulsesTargetStart = 0;
		encoderLeftPulsesTarget = 0;
		encoderLeftPulsesOnTarget = false;
		encoderPulsesTargetEnabled = false;
		}
	}
	
  	// set speed  
	// if(setPwmStatus) {    
	// 	leftMotorPwmOut = leftMotorPwmOutCmd;
	//  setPwmStatus = false;
	// } else {
    leftMotorPwmOut = 0;
    if(leftSpeedPidSetPoint != 0) {
      leftMotorPwmOut = leftSpeedPidOutput * leftSpeedPidSetPointDirection; // - steeringPidOutput;
    }
	//
	//  }
  
  	update_PID();

  	bodyMotorsControl();

	if(millis() >= secondTimer) {
		secondTimer = millis() + 1000;
		speedOs = (encoderLeftPulses - speedOsLastPulses) * mpp;
		speedOsLastPulses = encoderLeftPulses;
	}

	if(millis() >= loopInfoTimer) {
		loopInfoTimer = millis() + 50;
		
		Serial.print(twist_angular);
		Serial.print("\t");
		Serial.print(steering_angle);
		Serial.print("\t");
		Serial.print(servo1Angle);
		Serial.print(" | \t");
		Serial.print(twist_linear);
		Serial.print("\t");
		Serial.print(velocity_target);
		Serial.print(" | \t");
		Serial.print(speedOs);
		Serial.print("\t");
		Serial.print(speed);
		Serial.print("\t");
		Serial.print(leftSpeedPidSetPoint);
		Serial.print("\t");
		Serial.print(leftSpeedPidInputLast);
		Serial.print("\t");
		Serial.print(encoderLeftPulses);
		Serial.print("\t");
		Serial.print(leftMotorPwmOut);
		Serial.print("\t");
		Serial.println(motorState);
		
	}

	// prove of life & ros comms
	blinkLED();
	nh.spinOnce();

}