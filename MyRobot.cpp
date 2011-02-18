#include "WPILib.h"
#include "Target.h"
#include "DashboardDataSender.h"

#include <math.h>

// Line light sensor constants
#define SENSOR_SEES_LINE 	true
#define SENSOR_NO_LINE		false

// Joystick constants
#define LEFT_STICK_X				1
#define LEFT_STICK_Y				2
#define RIGHT_STICK_X				3
#define RIGHT_STICK_Y				4

//States for line tracking.
#define LT_FIND_LINE				0
#define LT_STRAFE_LEFT				1
#define LT_STRAFE_RIGHT				2
#define LT_DRIVE_FORWARD			3

// Camera target constants
#define MINIMUM_SCORE 0.01

/*
 * This code adds multitasking with turning.
 */

class RobotDemo : public SimpleRobot {
	Joystick driveControl;
	Joystick armControl;

	Jaguar frontRightJag;// Mecanum wheels =D
	Jaguar frontLeftJag;
	Jaguar backRightJag;
	Jaguar backLeftJag;
	Jaguar armLiftA;
	Jaguar armLiftB;
	Jaguar armDrum;
	DigitalInput lightSensorLeft; //Light sensor located at the front.
	DigitalInput lightSensorMiddle;
	DigitalInput lightSensorRight;
	DashboardDataSender dds;
	Gyro lineParallel;
	Ultrasonic wallSensor;
	int lt_state; //State of the line tracker.

public:
	RobotDemo(void) :
		driveControl(1),
		armControl(2),
		frontRightJag(3),
		frontLeftJag(2),
		backRightJag(4),
		backLeftJag(1),
		armLiftA(5),
		armLiftB(6),
		armDrum(7),
		lightSensorLeft(1),
		lightSensorMiddle(2),
		lightSensorRight(3),
		dds(),
		lt_state(LT_FIND_LINE),
		lineParallel(1),
		wallSensor(5,4)
	{
		Watchdog().SetExpiration(.75);
		wallSensor.SetAutomaticMode(true);
	}

	void Autonomous(void) {

		float a; //Front left.
		float b; //Front right.
		float c; //Back left.
		float d; //Back right.

		float hori1 = 0;
		float vert1 = 0;
		float hori2 = 0;

		float x;
		float y;
		float z;

		//While distance > value
		while (wallSensor.GetRangeMM() > 500) {
			switch (lt_state) {
			case LT_FIND_LINE: {
				if (lightSensorMiddle.Get() == SENSOR_SEES_LINE) {
					//Robot is on and parallel to line.
					lt_state = LT_DRIVE_FORWARD;
				} else if (lightSensorLeft.Get() == SENSOR_SEES_LINE) {
					//Robot is parallel, but slightly to the right of line.
					lt_state = LT_STRAFE_LEFT;
				} else if (lightSensorRight.Get() == SENSOR_SEES_LINE) {
					//Robot is parllel, but slightly to the left of line.
					lt_state = LT_STRAFE_RIGHT;
				}
				else 
				{
					hori1 += .5;
				}
			}
				break;
			case LT_STRAFE_LEFT: {
				if (lightSensorMiddle.Get() == SENSOR_SEES_LINE) {
					//Robot has succesfully strafed onto the line.
					lt_state = LT_DRIVE_FORWARD;
				} else {
					//Robot strafes to get onto line.
					hori1 = hori1 - .5;
				}
			}

				break;
			case LT_STRAFE_RIGHT: {
				if (lightSensorMiddle.Get() == SENSOR_SEES_LINE) {
					//Robot has succesfully gotten onto line.
					lt_state = LT_DRIVE_FORWARD;
				} else {
					//Robot strafes onto line.
					hori1 = hori1 + .5;
				}
			}
				break;
			case LT_DRIVE_FORWARD: {
				if (lightSensorLeft.Get() == SENSOR_SEES_LINE) {
					//Robot is slightly to the right of line.
					lt_state = LT_STRAFE_LEFT;
				} else if (lightSensorRight.Get() == SENSOR_SEES_LINE) {
					//Robot is slightly to the left of line.
					lt_state = LT_STRAFE_RIGHT;
				} else {
					vert1 = .5;
				}
			}
				break;
			default:
				lt_state = LT_FIND_LINE;
			}

			if (hori1 > x) {
				x = x + .0005;
			} else if (hori1 < x) {
				x = x - .0005;
			} else {
				x = hori1;
			}
			//Prossesing y1
			if (vert1 > y) {
				y = y + .0005;
			} else if (vert1 < y) {
				y = y - .0005;
			} else {
				y = vert1;
			}
			//Prossesing x2
			if (hori2 > z) {
				z = z + .0005;
			} else if (hori2 < z) {
				z = z - .0005;
			} else {
				z = hori2;
			}

			// Calculate motor control values based on joystick input
			//Upper Left Quadrant
			if (x < 0 && y < 0) {
				if (z > 0) {
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

				if (z > 0) {
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
			} else if (x> 0 && y > 0) {

				if (z > 0) {
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
			} else if (x < 0 && y > 0){

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

	}
		frontLeftJag.Set(a);
		frontRightJag.Set(b);
		backLeftJag.Set(c);
		backRightJag.Set(d);

			//todo: Arm code here.

}

void OperatorControl(void) {

	Watchdog().SetEnabled(true);
	wallSensor.SetEnabled(true);

	printf("Getting camera instance\n");
	AxisCamera &camera = AxisCamera::GetInstance();
	/**axis1	= x on left stick
	 ***axis2	= y on left stick
	 ***axis3	= x on right stick
	 ***axis4	= y on right stick
	 **/

	//Creates values for motors
	float a; //Front left.
	float b; //Front right.
	float c; //Back left.
	float d; //Back right.
	float rotation;
	lineParallel.Reset();

	while (IsOperatorControl()) {
		Watchdog().Feed();

		float x;
		float y;
		float z;

		rotation = fmod(lineParallel.GetAngle(),360.0) ;
		if (rotation>180)
		{
			rotation-=360;
		}
		else if (rotation<-180)
		{
			rotation+=360;
		}
		//Raw joystick inputs.
		float hori1 = 0;
		float vert1 = 0;
		float hori2 = 0;

		hori1 = driveControl.GetRawAxis(LEFT_STICK_X);
		vert1 = driveControl.GetRawAxis(LEFT_STICK_Y);
		hori2 = driveControl.GetRawAxis(RIGHT_STICK_X);

		// Motor control values
		a = 0; // front left
		b = 0; // front right
		c = 0; // back left
		d = 0; // back right

		// Test line tracking by strafing left and right
		//and rotating to stay on and parallel with the line.
		// Only enabled when button 6 pressed
		if (driveControl.GetRawButton(9) == 1)
		{
			printf("%f\n",wallSensor.GetRangeMM());
		}
		if (driveControl.GetRawButton(10))
		{
			lineParallel.Reset();
		}
		if (driveControl.GetRawButton(5))
		{
			if(rotation < 0)
			{
				hori2 += -.20 + .55 * (rotation / 360);
			}
			if(rotation > 0)
			{
				hori2 += .20 +.55 * (rotation / 360);
			}
		}
		
		if (driveControl.GetRawButton(6)) {
			printf("%f",hori2);
			switch (lt_state) {
				case LT_FIND_LINE: {
					if (lightSensorMiddle.Get() == SENSOR_SEES_LINE) {
						//Robot is on and parallel to line.
						lt_state = LT_DRIVE_FORWARD;
					} else if (lightSensorLeft.Get() == SENSOR_SEES_LINE) {
						//Robot is parallel, but slightly to the right of line.
						lt_state = LT_STRAFE_LEFT;
					} else if (lightSensorRight.Get() == SENSOR_SEES_LINE) {
						//Robot is parllel, but slightly to the left of line.
						lt_state = LT_STRAFE_RIGHT;
					}
				}
				break;
				case LT_STRAFE_LEFT: {
					if (lightSensorMiddle.Get() == SENSOR_SEES_LINE) {
						//Robot has succesfully strafed onto the line.
						lt_state = LT_DRIVE_FORWARD;
					}
					else {
						//Robot strafes to get onto line.
						hori1 = hori1 - .5;
					}
				}

				break;
				case LT_STRAFE_RIGHT: {
					if (lightSensorMiddle.Get() == SENSOR_SEES_LINE) {
						//Robot has succesfully gotten onto line.
						lt_state = LT_DRIVE_FORWARD;
					}
					else if (lightSensorLeft.Get() == SENSOR_SEES_LINE)
					{
						lt_state = LT_STRAFE_LEFT;
					}
					else {
						//Robot strafes onto line.
						hori1 = hori1 + .5;
					}
				}
				break;
				case LT_DRIVE_FORWARD: {
					if (lightSensorLeft.Get() == SENSOR_SEES_LINE) {
						//Robot is slightly to the right of line.
						lt_state = LT_STRAFE_LEFT;
					} else if (lightSensorRight.Get() == SENSOR_SEES_LINE) {
						//Robot is slightly to the left of line.
						lt_state = LT_STRAFE_RIGHT;
					}
					else if (lightSensorMiddle.Get() == SENSOR_NO_LINE)
					{
						lt_state = LT_STRAFE_RIGHT;
					}
				}
				break;
				default:
				lt_state = LT_FIND_LINE;
			}

		}

		else
		{
			switch(lt_state)
			{
				default:
				lt_state = LT_FIND_LINE;
			}
		}

		//Makes axes easier to understand. Processed inputs for slow acceleration.
		//Processing x1
		if (hori1> x) {
			x = x + .0005;
		} else if (hori1 < x) {
			x = x - .0005;
		} else {
			x = hori1;
		}
		//Prossesing y1
		if (vert1> y) {
			y = y + .0005;
		} else if (vert1 < y) {
			y = y - .0005;
		} else {
			y = vert1;
		}
		//Prossesing x2
		if (hori2> z) {
			z = z + .0005;
		} else if (hori2 < z) {
			z = z - .0005;
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
		} else if (x < 0 && y> 0) {

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
		
		//For minibot deployment.
		if (driveControl.GetRawButton(4) == 1)
		{
			a = .4;
			b = -.4;
			c = .4;
			d = -.4;
		}

		// Send the control values to the motor controllers
		frontLeftJag.Set(a);
		frontRightJag.Set(b);
		backLeftJag.Set(c);
		backRightJag.Set(d);

		//arm control
		if(armControl.GetRawButton(2))
		{
			armLiftA.Set(.5);
		}
		else if(armControl.GetRawButton(4))
		{
			armLiftA.Set(-.5);
		}
		else
		{
			armLiftA.Set(0);
		}
		if(armControl.GetRawAxis(6)> 0)
		{
			armLiftB.Set(.5);
		}
		else if(armControl.GetRawAxis(6) < 0)
		{
			armLiftB.Set(-.5);
		}
		else
		{
			armLiftB.Set(0);
		}
//todo: Make primary arm code.
		}

	}
}; // end OperatorControl()


START_ROBOT_CLASS(RobotDemo)
;
