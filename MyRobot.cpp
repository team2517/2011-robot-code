#include "WPILib.h"
#include "Target.h"
#include "DashboardDataSender.h"

#include <math.h>

//This part just lets the computer know that when I refer to the stuff on the left, I actually mean
//stuff on the right.
#define SENSOR_SEES_LINE 	true  //The light sensor sees the line.
#define SENSOR_NO_LINE		false  //The light sensor doesn't see the line.
#define CAN_LIFT 			true //The lift can move legally.
#define NO_LIFT 			false//The lift can't move legally.
#define CAN_TILT			true //The tilt can move legaly.
#define NO_TILT				false//The tilt can't move legaly.
// Joystick constants
#define LEFT_STICK_X				1
#define LEFT_STICK_Y				2
#define RIGHT_STICK_X				3
#define RIGHT_STICK_Y				4

//States for line tracking.
#define LT_FIND_LINE				0 //The robot has no idea where the line is.
#define LT_STRAFE_LEFT				1 //The robot needs to strafe left to follow the line.
#define LT_STRAFE_RIGHT				2 //The robot needs to strafe right to find the line.
#define LT_DRIVE_FORWARD			3 //The robot is centered on the line.
class RobotDemo : public SimpleRobot {
	Joystick driveControl; //The logitech dual stick
	Joystick armControl; //

	Jaguar frontRightJag;// Mecanum wheels with bearings pointing to the center when viewed from above.
	Jaguar frontLeftJag;
	Jaguar backRightJag;
	Jaguar backLeftJag;
	Relay miniA; //Minibot deployment pneumatic.
	Solenoid tiltA; //Pneumatic at the base of the arm that controls tilt.
	Solenoid tiltB;
	Relay tiltHold;
	Solenoid liftA; //The pneuamtic at the first joint that controls lift.
	Solenoid liftB;
	Relay liftHold;
	Solenoid clampA; //The pneumatic at the end of the arm that controls the clamp.
	Solenoid clampB;
	Compressor compress1; //The one and only air compressor.
	DigitalInput lightSensorLeft; //Light sensor that are located under a bar at the front bottom of
	DigitalInput lightSensorMiddle; //the robot.
	DigitalInput lightSensorRight;
	DashboardDataSender dds; //Sends data, such as the camera, to the dashboard.
	Gyro lineParallel; //Gyro sensor located on the electrical board in the center of the robot.
	Ultrasonic wallSensor; //Ultrasonic sensor located on the front left of the robot.
	int lt_state; //State of the line tracker.

public:
	RobotDemo(void) :
		driveControl(1), armControl(2), frontRightJag(3), frontLeftJag(2),
				backRightJag(4), backLeftJag(1), lightSensorLeft(1),
				lightSensorMiddle(2), lightSensorRight(3), dds(),
				lt_state(LT_FIND_LINE), lineParallel(1), wallSensor(5, 4),
				miniA(2, Relay::kForwardOnly),
				liftHold(4, Relay::kForwardOnly), tiltHold(3,
						Relay::kForwardOnly), tiltA(6), tiltB(7), liftA(4),
				liftB(5), clampA(2), clampB(3), compress1(6, 1)
	//Swapped tilt(2,3) and clamp (6,7)


	/* Joysticks (USB port)
	 * Jaguars (PWM)
	 * Solenoids (Solenoid Breakout Port)
	 * Compressor (Pressure Switch Digital i/o, Compressor slot.)
	 * DigitalInput (digital i/o)
	 * Gyro (analog input)
	 * Ultrasonic (analog input)
	 */
	{
		Watchdog().SetExpiration(.75); //Set watchdog timer to .75 seconds.
		wallSensor.SetAutomaticMode(true); //Start the wallSensor sending out sound automatically.
		compress1.Start(); //Start compressor compressing.

		liftA.Set(true);
		liftB.Set(false);
		tiltA.Set(true);
		tiltB.Set(false);
		miniA.Set(Relay::kOff);
		liftHold.Set(Relay::kOff);
		tiltHold.Set(Relay::kOff);
	}

	void Autonomous(void) {
		/*
		 float a = 0; //Creats and sets final motor output to zero.
		 float b = 0;
		 float c = 0;
		 float d = 0;

		 //float rotation = 0; //Creates and sets degrees in rotation to zero.
		 //lineParallel.Reset(); //Sets current gyro direction to zero.

		 float x; //Creates semiprocessessed, slow accelerating variable.
		 float y; // 
		 float z;

		 float hori1 = 0; //Creates raw controller outputs and sets them to zero.
		 float vert1 = 0;
		 float hori2 = 0;

		 frontRightJag.Set(-.4); //Robot suddenly moves forward...
		 frontLeftJag.Set(.4);
		 backRightJag.Set(-.4);
		 backLeftJag.Set(.4);

		 Wait(.75); // ...for a quarter of a second...

		 frontRightJag.Set(0); // ...and stops suddenly.
		 frontLeftJag.Set(0);
		 backRightJag.Set(0);
		 backLeftJag.Set(0);

		 Wait(.25);

		 //Starting position for arm.
		 tiltA.Set(true);
		 tiltB.Set(false);
		 liftA.Set(false);
		 liftB.Set(true);
		 clampA.Set(true);
		 clampB.Set(false);
		 miniA.Set(Relay::kOff);

		 while (wallSensor.GetRangeMM()> 500) { //This causes robot to stop when it gets within .5
		 //meters from the wall.

		 hori1 = 0; //Sets raw controller outputs to 0.
		 vert1 = 0;
		 hori2 = 0;

		 a = 0; //Sets final motor outputs.
		 b = 0;
		 c = 0;
		 d = 0;

		 miniA.Set(Relay::kOff);

		 //rotation = fmod(lineParallel.GetAngle(), 360.0); //Since rotating 360 degrees leaves you 
		 //facing the same direction,
		 //we just need the remainder.
		 //Causes the robot to rotate the other direction if the robot has rotated over 180 degrees
		 //in the previous direction.
		 //if (rotation>180) {
		 //rotation-=360;
		 //} else if (rotation<-180) {
		 //rotation+=360;
		 //}

		 //Modifies raw controller outputs to rotate starting fast and then slowing for accuracy.
		 //if (rotation < 0) {
		 //hori2 += -.20 + .55 * (rotation / 360);
		 //}
		 //if (rotation> 0) {
		 //hori2 += .20 +.55 * (rotation / 360);
		 //}

		 switch (lt_state) {
		 case LT_FIND_LINE: { //If robot has no idea where line is.
		 if (lightSensorMiddle.Get() == SENSOR_SEES_LINE) { //Look for middle sensor to see 
		 //line.
		 //Robot is on and parallel to line.
		 lt_state = LT_DRIVE_FORWARD;
		 } else if (lightSensorLeft.Get() == SENSOR_SEES_LINE) { //Wait for left sensor to see
		 //line.
		 //Robot is parallel, but slightly to the right of line.
		 lt_state = LT_STRAFE_LEFT;
		 } else if (lightSensorRight.Get() == SENSOR_SEES_LINE) { //Wait for right sensor to see
		 //line.
		 //Robot is parllel, but slightly to the left of line.
		 lt_state = LT_STRAFE_RIGHT;
		 }

		 }
		 break;
		 case LT_STRAFE_LEFT: {
		 if (lightSensorMiddle.Get() == SENSOR_SEES_LINE) { //Wait for middle sensor to see
		 //line.
		 //Robot has succesfully strafed onto the line.
		 lt_state = LT_DRIVE_FORWARD;
		 } else {
		 //Robot strafes to get onto line.
		 hori1 = hori1 - .5;
		 vert1 -= .5;
		 }
		 }

		 break;
		 case LT_STRAFE_RIGHT: {
		 if (lightSensorMiddle.Get() == SENSOR_SEES_LINE) { //Wait for middle sensor to see
		 //line.
		 //Robot has succesfully gotten onto line.
		 lt_state = LT_DRIVE_FORWARD;
		 } else if (lightSensorLeft.Get() == SENSOR_SEES_LINE) { //Wait for left sensor to see
		 //line.
		 //Robot is parallel but to the right of the line.
		 lt_state = LT_STRAFE_LEFT;
		 } else {
		 //Robot strafes onto line.
		 hori1 = hori1 + .5;
		 vert1 -=.5;
		 }
		 }
		 break;
		 case LT_DRIVE_FORWARD: {
		 if (lightSensorLeft.Get() == SENSOR_SEES_LINE) { //Wait for left sensor to see line.
		 //Robot is slightly to the right of line.
		 lt_state = LT_STRAFE_LEFT;
		 } else if (lightSensorRight.Get() == SENSOR_SEES_LINE) { //Wait for right sensor to
		 //see line.
		 //Robot is slightly to the left of line.
		 lt_state = LT_STRAFE_RIGHT;
		 }

		 else {
		 //Drive forward.
		 vert1 -= .5;
		 }
		 }
		 break;
		 default:
		 lt_state = LT_FIND_LINE;

		 }

		 //This would be the piece for slow acceleration.  However, we are moving so slowly in
		 //autonomous mode that we don't need it.  It would only make reactions sluggish.
		 x = hori1;
		 y = vert1;
		 z = hori2;

		 // Calculate motor control values based on joystick input
		 //Upper Left Quadrant
		 if (x < 0 && y < 0) {
		 if (z> 0) {
		 a = (-(x*x)+(y*y)+(z*z))/(-x-y+z);
		 b = (-(x*x)-(y*y)+(z*z))/(-x-y+z);
		 c = ((x*x)+(y*y)+(z*z))/(-x-y+z);
		 d = ((x*x)-(y*y)+(z*z))/(-x-y+z);
		 } else if (z < 0) {
		 a = (-(x*x)+(y*y)-(z*z))/(-x-y-z);
		 b = (-(x*x)-(y*y)-(z*z))/(-x-y-z);
		 c = ((x*x)+(y*y)-(z*z))/(-x-y-z);
		 d = ((x*x)-(y*y)-(z*z))/(-x-y-z);
		 } else {
		 a = (-(x*x)+(y*y)+(z*z))/(-x-y+z);
		 b = (-(x*x)-(y*y)+(z*z))/(-x-y+z);
		 c = ((x*x)+(y*y)+(z*z))/(-x-y+z);
		 d = ((x*x)-(y*y)+(z*z))/(-x-y+z);
		 }
		 //Upper Right Quadrant
		 } else if (x> 0 && y < 0) {

		 if (z> 0) {
		 a = ((x*x)+(y*y)+(z*z)) / (x-y+z);
		 b = ((x*x)-(y*y)+(z*z)) / (x-y+z);
		 c = (-(x*x)+(y*y)+(z*z)) / (x-y+z);
		 d = (-(x*x)-(y*y)+(z*z)) / (x-y+z);
		 }

		 else if (z < 0) {
		 a = ((x*x)+(y*y)-(z*z)) / (x-y-z);
		 b = ((x*x)-(y*y)-(z*z)) / (x-y-z);
		 c = (-(x*x)+(y*y)-(z*z)) / (x-y-z);
		 d = (-(x*x)-(y*y)-(z*z)) / (x-y-z);
		 }

		 else {
		 a = ((x*x)+(y*y)-(z*z)) / (x-y+z);
		 b = ((x*x)-(y*y)-(z*z)) / (x-y+z);
		 c = (-(x*x)+(y*y)-(z*z)) / (x-y+z);
		 d = (-(x*x)-(y*y)-(z*z)) / (x-y+z);
		 }
		 //Back Right Quadrant
		 } else if (x> 0 && y> 0) {

		 if (z> 0) {
		 a = ((x*x)-(y*y)+(z*z))/(x+y+z);
		 b = ((x*x)+(y*y)+(z*z))/(x+y+z);
		 c = (-(x*x)-(y*y)+(z*z))/(x+y+z);
		 d = (-(x*x)+(y*y)+(z*z))/(x+y+z);
		 }

		 else if (z < 0) {
		 a = ((x*x)-(y*y)-(z*z))/(x+y-z);
		 b = ((x*x)+(y*y)-(z*z))/(x+y-z);
		 c = (-(x*x)-(y*y)-(z*z))/(x+y-z);
		 d = (-(x*x)+(y*y)-(z*z))/(x+y-z);
		 } else {
		 a = ((x*x)-(y*y)+(z*z))/(x+y+z);
		 b = ((x*x)+(y*y)+(z*z))/(x+y+z);
		 c = (-(x*x)-(y*y)+(z*z))/(x+y+z);
		 d = (-(x*x)+(y*y)+(z*z))/(x+y+z);
		 }
		 //Back Left Quadrant
		 } else if (x < 0 && y> 0){

		 if (z> 0) {
		 a = (-(x*x)-(y*y)+(z*z))/(-x+y+z);
		 b = (-(x*x)+(y*y)+(z*z))/(-x+y+z);
		 c = ((x*x)-(y*y)+(z*z))/(-x+y+z);
		 d = ((x*x)+(y*y)+(z*z))/(-x+y+z);
		 }

		 else if (z < 0) {
		 a = (-(x*x)-(y*y)-(z*z))/(-x+y-z);
		 b = (-(x*x)+(y*y)-(z*z))/(-x+y-z);
		 c = ((x*x)-(y*y)-(z*z))/(-x+y-z);
		 d = ((x*x)+(y*y)-(z*z))/(-x+y-z);
		 }
		 else
		 {
		 a = (-(x*x)-(y*y)+(z*z))/(-x+y+z);
		 b = (-(x*x)+(y*y)+(z*z))/(-x+y+z);
		 c = ((x*x)-(y*y)+(z*z))/(-x+y+z);
		 d = ((x*x)+(y*y)+(z*z))/(-x+y+z);
		 }
		 }
		 //Strafe Right
		 else if (x> 0 && y == 0)
		 {
		 if(z> 0)
		 {
		 a = ((x*x)+(z*z))/(x+z);
		 b = ((x*x)+(z*z))/(x+z);
		 c = (-(x*x)+(z*z))/(x+z);
		 d = (-(x*x)+(z*z))/(x+z);
		 }
		 else if(z < 0)
		 {
		 a = ((x*x)-(z*z))/(x-z);
		 b = ((x*x)-(z*z))/(x-z);
		 c = (-(x*x)-(z*z))/(x-z);
		 d = (-(x*x)-(z*z))/(x-z);
		 }
		 else
		 {
		 a = ((x*x)+(z*z))/(x+z);
		 b = ((x*x)+(z*z))/(x+z);
		 c = (-(x*x)+(z*z))/(x+z);
		 d = (-(x*x)+(z*z))/(x+z);
		 }
		 }//
		 //Strafe Left
		 else if (x < 0 && y == 0)
		 {
		 if(z> 0)
		 {
		 a = (-(x*x)+(z*z))/(-x+z);
		 b = (-(x*x)+(z*z))/(-x+z);
		 c = ((x*x)+(z*z))/(-x+z);
		 d = ((x*x)+(z*z))/(-x+z);
		 }
		 else if(z < 0)
		 {
		 a = (-(x*x)-(z*z))/(-x-z);
		 b = (-(x*x)-(z*z))/(-x-z);
		 c = ((x*x)-(z*z))/(-x-z);
		 d = ((x*x)-(z*z))/(-x-z);
		 }
		 else
		 {
		 a = (-(x*x)+(z*z))/(-x+z);
		 b = (-(x*x)+(z*z))/(-x+z);
		 c = ((x*x)+(z*z))/(-x+z);
		 d = ((x*x)+(z*z))/(-x+z);
		 }
		 }
		 //Backward
		 else if(y> 0 && x == 0)
		 {
		 if (z> 0)
		 {
		 a = (-(y*y)+(z*z))/(y+z);
		 b = ((y*y)+(z*z))/(y+z);
		 c = (-(y*y)+(z*z))/(y+z);
		 d = ((y*y)+(z*z))/(y+z);
		 }
		 else if (z < 0)
		 {
		 a = (-(y*y)-(z*z))/(y-z);
		 b = ((y*y)-(z*z))/(y-z);
		 c = (-(y*y)-(z*z))/(y-z);
		 d = ((y*y)-(z*z))/(y-z);
		 }
		 else
		 {
		 a = (-(y*y)+(z*z))/(y+z);
		 b = ((y*y)+(z*z))/(y+z);
		 c = (-(y*y)+(z*z))/(y+z);
		 d = ((y*y)+(z*z))/(y+z);
		 }
		 }
		 //Forward
		 else if (y < 0 && x == 0)
		 {
		 if (z> 0)
		 {
		 a = ((y*y)+(z*z))/(-y+z);
		 b = (-(y*y)+(z*z))/(-y+z);
		 c = ((y*y)+(z*z))/(-y+z);
		 d = (-(y*y)+(z*z))/(-y+z);
		 }
		 else if (z < 0)
		 {
		 a = ((y*y)-(z*z))/(-y-z);
		 b = (-(y*y)-(z*z))/(-y-z);
		 c = ((y*y)-(z*z))/(-y-z);
		 d = (-(y*y)-(z*z))/(-y-z);
		 }
		 else
		 {
		 a = ((y*y)-(z*z))/(-y+z);
		 b = (-(y*y)-(z*z))/(-y+z);
		 c = ((y*y)-(z*z))/(-y+z);
		 d = (-(y*y)-(z*z))/(-y+z);
		 }
		 }

		 // Send the control values to the motor controllers
		 frontLeftJag.Set(a);
		 frontRightJag.Set(b);
		 backLeftJag.Set(c);
		 backRightJag.Set(d);

		 }
		 //Stop moving forward.
		 frontLeftJag.Set(0);
		 frontRightJag.Set(0);
		 backLeftJag.Set(0);
		 backRightJag.Set(0);

		 liftA.Set(true); //Lower second joint of arm.
		 liftB.Set(false);
		 clampA.Set(false); //Release clamp.
		 clampB.Set(true);

		 Wait(.1);

		 frontRightJag.Set(.4);
		 frontLeftJag.Set(-.4);
		 backRightJag.Set(.4);
		 backLeftJag.Set(-.4);

		 Wait(.5);

		 frontRightJag.Set(0);
		 frontLeftJag.Set(0);
		 backRightJag.Set(0);
		 backLeftJag.Set(0);
		 
		 */
	}

	void OperatorControl(void) {

		Watchdog().SetEnabled(true);

		//Send camera image to classmate.
		printf("Getting camera instance\n");
		AxisCamera &camera = AxisCamera::GetInstance();
		/**axis1	= x on left stick
		 ***axis2	= y on left stick
		 ***axis3	= x on right stick
		 ***axis4	= y on right stick
		 **/

		//Creates values for motors
		float a = 0; //Create and set final motor outputs to zero.
		float b = 0;
		float c = 0;
		float d = 0;
		float rotation = 0; //Create and set degrees rotated to zero.
		float x = 0; //Create and set semi-proccessed slow accelerating motor outputs to zero.
		float y = 0;
		float z = 0;
		float hori1 = 0; //Create and set raw controller outputs to zero.
		float vert1 = 0;
		float hori2 = 0;

		bool liftok= CAN_LIFT;
		bool tiltok= CAN_TILT;

		liftA.Set(true);
		liftB.Set(false);
		tiltA.Set(true);
		tiltB.Set(false);
		miniA.Set(Relay::kOn);

		while (IsOperatorControl()) {
			Watchdog().Feed();

			rotation = fmod(lineParallel.GetAngle(), 360.0); //Since turning 360 degrees keeps you facing
			//the same direction, we only need the 
			//remainder.
			//To turn the least, after turning over 180 degrees, you keep turning the same direction to 
			//right yourself.
			if (rotation>180) {
				rotation-=360;
			} else if (rotation<-180) {
				rotation+=360;
			}
			//Raw joystick inputs set to zero.
			hori1 = 0;
			vert1 = 0;
			hori2 = 0;
			//Set raw joystick outputs to variables.
			hori1 = driveControl.GetRawAxis(LEFT_STICK_X);
			vert1 = driveControl.GetRawAxis(LEFT_STICK_Y);
			hori2 = driveControl.GetRawAxis(RIGHT_STICK_X);
			//Reduces movement speed for precise control when button eight is pressed.
			if (driveControl.GetRawButton(8)) {
				hori1 = hori1 * .25;
				vert1 = vert1 * .25;
				hori2 = hori2 * .25;
			}

			// Motor control values
			a = 0; // front left
			b = 0; // front right
			c = 0; // back left
			d = 0; // back right

			// Test line tracking by strafing left and right
			//and rotating to stay on and parallel with the line.
			// Only enabled when button 6 pressed
			if (driveControl.GetRawButton(10)) {
				lineParallel.Reset(); //Set current direction to zero when button 10 is pressed.
			}
			if (driveControl.GetRawButton(5)) {
				//Rotate and right the robot at a speed based on degrees changed when button five is
				//pressed.
				if (rotation < 0) {
					hori2 += -.20 + .55 * (rotation / 360);
				}
				if (rotation> 0) {
					hori2 += .20 +.55 * (rotation / 360);
				}
			}
			printf("%f\n", lightSensorLeft.Get());
			if (driveControl.GetRawButton(6)) {
				//Enabe line tracking when button six is pressed.
				switch (lt_state) {
				case LT_FIND_LINE: {
					if (lightSensorMiddle.Get() == SENSOR_SEES_LINE) { //Wait for middle sensor to see
						//line.
						//Robot is on and parallel to line.
						lt_state = LT_DRIVE_FORWARD;
					} else if (lightSensorLeft.Get() == SENSOR_SEES_LINE) { //Wait for left sensor to 
						//see line.
						//Robot is parallel, but slightly to the right of line.
						lt_state = LT_STRAFE_LEFT;
					} else if (lightSensorRight.Get() == SENSOR_SEES_LINE) { //Wait for right sensor to
						//see line.
						//Robot is parllel, but slightly to the left of line.
						lt_state = LT_STRAFE_RIGHT;
					}
				}
					break;
				case LT_STRAFE_LEFT: {
					if (lightSensorMiddle.Get() == SENSOR_SEES_LINE) { //Wait for middle sensor to see
						//line.
						//Robot has succesfully strafed onto the line.
						lt_state = LT_DRIVE_FORWARD;
					} else {
						//Robot strafes to get onto line.
						hori1 = hori1 - .5;
					}
				}

					break;
				case LT_STRAFE_RIGHT: {
					if (lightSensorMiddle.Get() == SENSOR_SEES_LINE) { //Wait for middle sensor to see
						//line.
						//Robot has succesfully gotten onto line.
						lt_state = LT_DRIVE_FORWARD;
					} else if (lightSensorLeft.Get() == SENSOR_SEES_LINE) //Wait for left sensor to see
					//line.
					{
						lt_state = LT_STRAFE_LEFT;
					} else {
						//Robot strafes onto line.
						hori1 = hori1 + .5;
					}
				}
					break;
				case LT_DRIVE_FORWARD: {
					if (lightSensorLeft.Get() == SENSOR_SEES_LINE) { //Wait for left sensor to see
						//line.
						//Robot is slightly to the right of line.
						lt_state = LT_STRAFE_LEFT;
					} else if (lightSensorRight.Get() == SENSOR_SEES_LINE) { //Wait for right sensor to
						//see line.
						//Robot is slightly to the left of line.
						lt_state = LT_STRAFE_RIGHT;
					}
				}
					break;
				default:
					lt_state = LT_FIND_LINE;
				}

			}

			else {
				switch (lt_state) {
				default:
					lt_state = LT_FIND_LINE;
				}
			}

			//Processing inputs to make for slow acceleration.
			if (hori1> x) {
				x = x +.05;
			} else if (hori1 < x) {
				x = x - .05;
			} else {
				x = hori1;
			}
			if (vert1> y) {
				y = y +.05;
			} else if (vert1 < y) {
				y = y - .05;
			} else {
				y = vert1;
			}
			if (hori2> z) {
				z = z +.05;
			} else if (hori2 < z) {
				z = z - .05;
			} else {
				z = hori2;
			}

			// Calculate motor control values based on joystick input
			//Upper Left Quadrant
			if (x < 0 && y < 0) {
				if (z> 0) {
					a = (-(x*x)+(y*y)+(z*z))/(-x-y+z);
					b = (-(x*x)-(y*y)+(z*z))/(-x-y+z);
					c = ((x*x)+(y*y)+(z*z))/(-x-y+z);
					d = ((x*x)-(y*y)+(z*z))/(-x-y+z);
				} else if (z < 0) {
					a = (-(x*x)+(y*y)-(z*z))/(-x-y-z);
					b = (-(x*x)-(y*y)-(z*z))/(-x-y-z);
					c = ((x*x)+(y*y)-(z*z))/(-x-y-z);
					d = ((x*x)-(y*y)-(z*z))/(-x-y-z);
				} else {
					a = (-(x*x)+(y*y)+(z*z))/(-x-y+z);
					b = (-(x*x)-(y*y)+(z*z))/(-x-y+z);
					c = ((x*x)+(y*y)+(z*z))/(-x-y+z);
					d = ((x*x)-(y*y)+(z*z))/(-x-y+z);
				}
				//Upper Right Quadrant
			} else if (x> 0 && y < 0) {

				if (z> 0) {
					a = ((x*x)+(y*y)+(z*z)) / (x-y+z);
					b = ((x*x)-(y*y)+(z*z)) / (x-y+z);
					c = (-(x*x)+(y*y)+(z*z)) / (x-y+z);
					d = (-(x*x)-(y*y)+(z*z)) / (x-y+z);
				}

				else if (z < 0) {
					a = ((x*x)+(y*y)-(z*z)) / (x-y-z);
					b = ((x*x)-(y*y)-(z*z)) / (x-y-z);
					c = (-(x*x)+(y*y)-(z*z)) / (x-y-z);
					d = (-(x*x)-(y*y)-(z*z)) / (x-y-z);
				}

				else {
					a = ((x*x)+(y*y)-(z*z)) / (x-y+z);
					b = ((x*x)-(y*y)-(z*z)) / (x-y+z);
					c = (-(x*x)+(y*y)-(z*z)) / (x-y+z);
					d = (-(x*x)-(y*y)-(z*z)) / (x-y+z);
				}
				//Back Right Quadrant
			} else if (x> 0 && y> 0) {

				if (z> 0) {
					a = ((x*x)-(y*y)+(z*z))/(x+y+z);
					b = ((x*x)+(y*y)+(z*z))/(x+y+z);
					c = (-(x*x)-(y*y)+(z*z))/(x+y+z);
					d = (-(x*x)+(y*y)+(z*z))/(x+y+z);
				}

				else if (z < 0) {
					a = ((x*x)-(y*y)-(z*z))/(x+y-z);
					b = ((x*x)+(y*y)-(z*z))/(x+y-z);
					c = (-(x*x)-(y*y)-(z*z))/(x+y-z);
					d = (-(x*x)+(y*y)-(z*z))/(x+y-z);
				} else {
					a = ((x*x)-(y*y)+(z*z))/(x+y+z);
					b = ((x*x)+(y*y)+(z*z))/(x+y+z);
					c = (-(x*x)-(y*y)+(z*z))/(x+y+z);
					d = (-(x*x)+(y*y)+(z*z))/(x+y+z);
				}
				//Back Left Quadrant
			} else if (x < 0 && y> 0){

			if (z> 0) {
				a = (-(x*x)-(y*y)+(z*z))/(-x+y+z);
				b = (-(x*x)+(y*y)+(z*z))/(-x+y+z);
				c = ((x*x)-(y*y)+(z*z))/(-x+y+z);
				d = ((x*x)+(y*y)+(z*z))/(-x+y+z);
			}

			else if (z < 0) {
				a = (-(x*x)-(y*y)-(z*z))/(-x+y-z);
				b = (-(x*x)+(y*y)-(z*z))/(-x+y-z);
				c = ((x*x)-(y*y)-(z*z))/(-x+y-z);
				d = ((x*x)+(y*y)-(z*z))/(-x+y-z);
			}
			else
			{
				a = (-(x*x)-(y*y)+(z*z))/(-x+y+z);
				b = (-(x*x)+(y*y)+(z*z))/(-x+y+z);
				c = ((x*x)-(y*y)+(z*z))/(-x+y+z);
				d = ((x*x)+(y*y)+(z*z))/(-x+y+z);
			}
		}
		//Strafe Right
		else if (x> 0 && y == 0)
		{
			if(z> 0)
			{
				a = ((x*x)+(z*z))/(x+z);
				b = ((x*x)+(z*z))/(x+z);
				c = (-(x*x)+(z*z))/(x+z);
				d = (-(x*x)+(z*z))/(x+z);
			}
			else if(z < 0)
			{
				a = ((x*x)-(z*z))/(x-z);
				b = ((x*x)-(z*z))/(x-z);
				c = (-(x*x)-(z*z))/(x-z);
				d = (-(x*x)-(z*z))/(x-z);
			}
			else
			{
				a = ((x*x)+(z*z))/(x+z);
				b = ((x*x)+(z*z))/(x+z);
				c = (-(x*x)+(z*z))/(x+z);
				d = (-(x*x)+(z*z))/(x+z);
			}
		}
		//Strafe Left
		else if (x < 0 && y == 0)
		{
			if(z> 0)
			{
				a = (-(x*x)+(z*z))/(-x+z);
				b = (-(x*x)+(z*z))/(-x+z);
				c = ((x*x)+(z*z))/(-x+z);
				d = ((x*x)+(z*z))/(-x+z);
			}
			else if(z < 0)
			{
				a = (-(x*x)-(z*z))/(-x-z);
				b = (-(x*x)-(z*z))/(-x-z);
				c = ((x*x)-(z*z))/(-x-z);
				d = ((x*x)-(z*z))/(-x-z);
			}
			else
			{
				a = (-(x*x)+(z*z))/(-x+z);
				b = (-(x*x)+(z*z))/(-x+z);
				c = ((x*x)+(z*z))/(-x+z);
				d = ((x*x)+(z*z))/(-x+z);
			}
		}
		//Backward
		else if(y> 0 && x == 0)
		{
			if (z> 0)
			{
				a = (-(y*y)+(z*z))/(y+z);
				b = ((y*y)+(z*z))/(y+z);
				c = (-(y*y)+(z*z))/(y+z);
				d = ((y*y)+(z*z))/(y+z);
			}
			else if (z < 0)
			{
				a = (-(y*y)-(z*z))/(y-z);
				b = ((y*y)-(z*z))/(y-z);
				c = (-(y*y)-(z*z))/(y-z);
				d = ((y*y)-(z*z))/(y-z);
			}
			else
			{
				a = (-(y*y)+(z*z))/(y+z);
				b = ((y*y)+(z*z))/(y+z);
				c = (-(y*y)+(z*z))/(y+z);
				d = ((y*y)+(z*z))/(y+z);
			}
		}
		//Forward
		else if (y < 0 && x == 0)
		{
			if (z> 0)
			{
				a = ((y*y)+(z*z))/(-y+z);
				b = (-(y*y)+(z*z))/(-y+z);
				c = ((y*y)+(z*z))/(-y+z);
				d = (-(y*y)+(z*z))/(-y+z);
			}
			else if (z < 0)
			{
				a = ((y*y)-(z*z))/(-y-z);
				b = (-(y*y)-(z*z))/(-y-z);
				c = ((y*y)-(z*z))/(-y-z);
				d = (-(y*y)-(z*z))/(-y-z);
			}
			else
			{
				a = ((y*y)-(z*z))/(-y+z);
				b = (-(y*y)-(z*z))/(-y+z);
				c = ((y*y)-(z*z))/(-y+z);
				d = (-(y*y)-(z*z))/(-y+z);
			}
		}

		// Send the control values to the motor controllers
		frontLeftJag.Set(a);
		frontRightJag.Set(b);
		backLeftJag.Set(c);
		backRightJag.Set(d);

		//Minibot Deployment
		if (armControl.GetRawButton(6))
		{
			miniA.Set(Relay::kOn);
		}
		else
		{
			miniA.Set(Relay::kOff);
		}

		//Arm Control
		if (tiltA.Get() == true && tiltB.Get() == false)
		{
			if (armControl.GetRawButton(3) && liftok)
			{
				liftA.Set(false); //Lift extends when button 3 is pressed and tilt is retracted.
				liftB.Set(true);
				tiltok = NO_TILT;
			}
			else if (armControl.GetRawButton(2))
			{
				liftA.Set(true);
				liftB.Set(false);
				tiltok = CAN_TILT;
			}
		}

		if (liftA.Get() == true && liftB.Get() == false)
		{
			if(armControl.GetRawButton(9) && tiltok)
			{
				tiltA.Set(false); //Tilt retracts when button 9 is pressed and lift is retracted.
				tiltB.Set(true);
				liftok = NO_LIFT;
			}
			else if (armControl.GetRawButton(8))
			{
				tiltA.Set(true);
				tiltB.Set(false);
				liftok = CAN_LIFT;
			}
		}

		if (armControl.GetRawButton(1))
		{
			clampA.Set(false); //Clamp opens when button 1 is pressed.
			clampB.Set(true);
			tiltHold.Set(Relay::kOn);
		}
		else
		{
			clampA.Set(true);
			clampB.Set(false);
		}

	}

}

}; // end OperatorControl()


START_ROBOT_CLASS(RobotDemo)
;

