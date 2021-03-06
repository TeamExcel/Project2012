#include "WPILib.h"
//#include "Robot2012.h"
#include "Vision/AxisCamera.h"
#include "nivision.h"
#include "ImageProcessing.h"
#include "customPIDs.h"
#include "AnalogRangeFinder.h"

////////////////////////////////////////////////////////
// Defines and typedefs
////////////////////////////////////////////////////////
#define ANALOG_OUTPUT_CHANNEL 1		//in 2012 this should be in slot 1 (chan 1), or slot 5 (chan 2)
#define DIGITAL_OUTPUT_CHANNEL 1	//in 2012 this should be in slot 2 (chan 1), or slot 7 (chan 2)
#define SOLENOID_OUTPUT_CHANNEL 1	//in 2012 this should be in slot 3 (chan 1), or slot 6 (chan 2)

#define CATAPULT_FIRE_TIME 1.0
#define CATAPULT_REARM_TIME 1.00
#define CATAPULT_RELOAD_TIME_FOUR_BALL 2.00
#define SLOW_PICKUP_DURING_RELOAD_START 0.25
#define CATAPULT_LATCH_TIME 0.2

//Loading from the floor occurs at times:  (note: ideally, drop at <1.0 and at 6.25)
//First ball time < 1.0 (CATAPULT_FIRE_TIME) sec or  3.0 (FIRE + REARM + SLOW_PICKUP) < time < 6.75 ( 1 FULL CYCLE + FIRE + REARM)
//Second ball 6.25 < time < 10.0 (previous times + CYCLE_TIME)

//Generically ((CYCLE_TIME * EXTRA_BALL_NUM) - (RELOAD_TIME - SLOW_PICKUP)) < time < ((CYCLE_TIME * EXTRA_BALL_NUM) + FIRE_TIME + REARM_TIME) 

//Ball fire times = 4.0 * (ball_num - 1)  (0,4,8,12,16,20)


#define AUTONOMOUS_BACKUP_SPEED_SLOW -0.45
#define AUTONOMOUS_BACKUP_SPEED_FAST -0.7
#define AUTONOMOUS_BACKUP_TIME_SLOW (2.1 + AUTONOMOUS_BACKUP_TIME_FAST)
#define AUTONOMOUS_BACKUP_TIME_FAST (1.0 + AUTONOMOUS_BACKUP_TIME_WAIT)
#define AUTONOMOUS_BACKUP_TIME_WAIT 1.0
#define AUTONOMOUS_FENDER_SHOT_FAST (-0.55)
#define AUTONOMOUS_FENDER_SHOT_SLOW (-0.3)
//#define CATAPULT_PRELOAD_TIME_FOUR_BALL 0.1  	//How long we turn on the reload roller before the catapult finishes rearming
//Fire 0.0sec
//Rearm start 0.0sec
//preload CATAPULT_REARM_TIME - CATAPULT_PRELOAD_TIME_FOUR_BALL
//reloading CATAPULT_REARM_TIME
//slow ball collect CATAPULT_REARM_TIME + 1.0
//fire again CATAPULT_REARM_TIME + CATAPULT_RELOAD_TIME_FOUR_BALL 



#define ANGLE_POSITION_BASE 0.0
#define ANGLE_POSITION_TOLERANCE 0.5
#define DISTANCE_POSITION_BASE 152.0
#define DISTANCE_POSITION_TOLERANCE 1.0

#define CAMERA_MAX_FPS 10
#define CAMERA_BRIGHTNESS_LEVEL 5
#define CAMERA_COLOR_LEVEL 100

#define VERY_SLOW_TURN 0.20F
#define SLOW_TURN 0.30F
#define FAST_TURN 0.40F

#define ELEVATOR_SPEED_BOTTOM 0.8F
#define ELEVATOR_SPEED_BOTTOM_SLOW 0.3F
#define ELEVATOR_SPEED_TOP 1.0F
#define DUMPER_ROLLER_RPM 4100.0F
#define DUMPER_ROLLER_POWER 0.725
#define DUMPER_ROLLER_COUNTS_PER_REVOLUTION (400)  //This is a property of the encoder we bought, don't change
#define DUMPER_ROLLER_FILTER_CONSTANT 0.1
#define DUMPER_ROLLER_VOLTAGE_RAMP_RATE 0.19


#define CENTER_OF_IMAGE 160




#define HALF_HORIZONTAL_FIELD_OF_VIEW 24.40828F
#define HORIZONTAL_DEGREES_PER_PIXEL (HALF_HORIZONTAL_FIELD_OF_VIEW/CENTER_OF_IMAGE)


#define HALF_VERTICAL_FIELD_OF_VIEW 19.13711F
#define TANGENT_OF_HALF_VERTICAL_FIELD_OF_VIEW .347007F
#define TARGET_HEIGHT_IN_FEET 1.5F
#define IMAGE_HEIGHT_IN_PIXELS 240
#define IMAGE_HEIGHT_IN_FEET(target_height_pxls) ((TARGET_HEIGHT_IN_FEET * (IMAGE_HEIGHT_IN_PIXELS / 2)) / target_height_pxls)
#define CALCULATE_DISTANCE(target_height_pxls) (IMAGE_HEIGHT_IN_FEET(target_height_pxls)/TANGENT_OF_HALF_VERTICAL_FIELD_OF_VIEW)

#define FPGA_TIME_TO_MINUTES_FACTOR (60 * 1000 * 1000) //FPGA time is in uSec

//Controls defines - for new buttons, add a #define here and use it to get the key you want, that way we can change controls easily
#define BUTTON_CAMERA_ALIGN_SHOT_BUTTON() stickRightDrive.GetTrigger()
#define BUTTON_CAMERA_TAKE_DEBUG_PICTURES() stickRightDrive.GetRawButton(3)

#define BUTTON_ELEVATOR_BOTTOM_UP() (stickShooter.GetRawButton(2) || stickRightDrive.GetTop())
#define BUTTON_ELEVATOR_BOTTOM_DOWN() stickShooter.GetRawButton(3)
#define BUTTON_ELEVATOR_TOP_UP() stickShooter.GetRawButton(4)
#define BUTTON_ELEVATOR_TOP_DOWN() stickShooter.GetRawButton(5)

#define BUTTON_CATAPULT_SHOOT() stickShooter.GetTrigger()
#define BUTTON_CATAPULT_LATCH() stickShooter.GetTrigger()
#define BUTTON_CATAPULT_FORCE_SHOOT() stickShooter.GetRawButton(10)
//#define THROTTLE_TOP_ROLLER() ((stickShooter.GetThrottle() + 1) / 2)
#define BUTTON_DUMPER_RAMP_EXTEND() stickShooter.GetRawButton(9)
#define BUTTON_DUMPER_ROLLER() stickShooter.GetRawButton(11)

#define BUTTON_BRIDGE_RAM_TOGGLE() stickLeftDrive.GetTop()
#define BUTTON_STINGER_TOGGLE() (stickLeftDrive.GetRawButton(5) && stickRightDrive.GetRawButton(4))

//#define BUTTON_COMBO_SWITCH_AUTONOMOUS() (stickShooter.GetTrigger() && stickLeftDrive.GetTrigger() && stickRightDrive.GetTrigger())
#define BUTTON_COMBO_SWITCH_AUTONOMOUS() (stickRightDrive.GetTrigger())
//Kinect defines
#define KINECT_HEAD_RIGHT() kinectLeft.GetRawButton(1)
#define KINECT_HEAD_LEFT() kinectLeft.GetRawButton(2)
#define KINECT_RIGHT_LEG_RIGHT() kinectLeft.GetRawButton(3)
#define KINECT_LEFT_LEG_LEFT() kinectLeft.GetRawButton(4)
#define KINECT_RIGHT_LEG_FORWARD() kinectLeft.GetRawButton(5)
#define KINECT_RIGHT_LEG_BACK() kinectLeft.GetRawButton(6)
#define KINECT_LEFT_LEG_FORWARD() kinectLeft.GetRawButton(7)
#define KINECT_LEFT_LEG_BACK() kinectLeft.GetRawButton(8)
#define KINECT_CONTROL_ENABLED() kinectLeft.GetRawButton(9)

#define KINECT_ELEVATORS_UP() KINECT_RIGHT_LEG_RIGHT()
#define KINECT_ELEVATORS_DOWN() KINECT_LEFT_LEG_LEFT()
#define KINECT_AUTONOMOUS_SHOOT() KINECT_HEAD_RIGHT()
#define KINECT_BRIDGE_RAM_EXTEND() KINECT_HEAD_LEFT()




//PID Parameters
#define ROTATION_PID_PROPORTION 0.26
#define ROTATION_PID_INTEGRAL 0.07
#define ROTATION_PID_DERIVATIVE 0.36

#define ROTATION_PID_MIN_INPUT -30.0
#define ROTATION_PID_MAX_INPUT 30.0
#define ROTATION_PID_MIN_OUTPUT -1.00
#define ROTATION_PID_MAX_OUTPUT 1.00

#define ROTATION_PID_TOLERENCE_FIRST 2.50
#define ROTATION_PID_TOLERENCE_LAST 0.50
#define ROTATION_PID_SETPOINT_OFFSET -1.0 //negative adjusts to the right

#define RANGE_PID_PROPORTION 0.02
#define RANGE_PID_INTEGRAL 0.0005
#define RANGE_PID_DERIVATIVE 0.005

#define RANGE_PID_MIN_INPUT 0.0
#define RANGE_PID_MAX_INPUT 240.0
#define RANGE_PID_MIN_OUTPUT -0.40
#define RANGE_PID_MAX_OUTPUT 0.40

#define RANGE_PID_SETPOINT 150.0
#define RANGE_PID_TOLERENCE 4.0

//Uncomment this to enable the PID tuning section of code that can help tune PIDs
//by running code in debug mode and using breakpoints.
//#define PID_TUNING



//Comment this out to allow range adjusting in autonomous
#define DISABLE_RANGE_ADJUST_IN_AUTONOMOUS
#define ENABLE_FOUR_SHOT_SUPER_AUTONOMOUS
#define DISABLE_RANGE_FINDER
#define LINING_UP_IN_AUTONOMOUS false
//#define TEST_ELEVATOR_TIMING


//Helper Macros
#define STINGER_EXTENDED(isExtended) {FlipSolenoids(isExtended, &solenoidStingerExtend, &solenoidStingerRetract);}
#define BRIDGE_RAM_EXTENDED(isExtended) {FlipSolenoids(isExtended, &solenoidBridgeRamDown, &solenoidBridgeRamUp);}
#define CATAPULT_PUSHER_EXTENDED(isExtended) {FlipSolenoids(isExtended, &solenoidCatapultPusher, &solenoidCatapultPuller);}
#define CATAPULT_LATCH_EXTENDED(isExtended) {FlipSolenoids(isExtended, &solenoidCatapultLatchExtend, &solenoidCatapultLatchRetract);}
#define DISABLE_PID(pid) {if (pid.IsEnabled()){pid.Disable();pid.Reset();}}
typedef enum
{
	PWM_CHANNEL_1_JAGUAR_REAR_RIGHT = 1,
	PWM_CHANNEL_2_JAGUAR_REAR_LEFT,
	PWM_CHANNEL_3_JAGUAR_FRONT_RIGHT,
	PWM_CHANNEL_4_JAGUAR_FRONT_LEFT,
	PWM_CHANNEL_5_JAGUAR_ELEVATOR_BOTTOM_1,
	PWM_CHANNEL_6_JAGUAR_ELEVATOR_BOTTOM_2,
	PWM_CHANNEL_7_JAGUAR_ELEVATOR_TOP,
	PWM_CHANNEL_8_JAGUAR_DUMPER_ROLLER,
	PWM_CHANNEL_9_UNUSED
}PWM_CHANNEL_TYPE;

#define BOTTOM_ROLLERS_SYNC_GROUP 1
 
typedef enum
{
	SOLENOID_CHANNEL_1_CATAPULT_PUSHER = 1,
	SOLENOID_CHANNEL_2_CATAPULT_PULLER,
	SOLENOID_CHANNEL_3_BRIDGE_RAM_DOWN,
	SOLENOID_CHANNEL_4_BRIDGE_RAM_UP,
	SOLENOID_CHANNEL_5_STINGER_RETRACT,
	SOLENOID_CHANNEL_6_STINGER_EXTEND,
	SOLENOID_CHANNEL_7_CATAPULT_LATCH_EXTEND,
	SOLENOID_CHANNEL_8_CATAPULT_LATCH_RETRACT
}SOLENOID_CHANNEL_TYPE;

typedef enum
{
	ANALOG_CHANNEL_1_GYROSCOPE = 1,
	ANALOG_CHANNEL_2_RANGE_FINDER,
	ANALOG_CHANNEL_3_UNUSED,
	ANALOG_CHANNEL_4_UNUSED,
	ANALOG_CHANNEL_5_UNUSED,
	ANALOG_CHANNEL_6_UNUSED,
	ANALOG_CHANNEL_7_UNUSED,
	ANALOG_CHANNEL_8_UNUSED,
	ANALOG_CHANNEL_9_UNUSED
}ANALOG_CHANNEL_TYPE;

typedef enum
{
	DIGITAL_CHANNEL_1_INPUT_COMPRESSOR_SWITCH = 1,
	DIGITAL_CHANNEL_2_DUMPER_COUNTER,
	DIGITAL_CHANNEL_3_UNUSED,
	DIGITAL_CHANNEL_4_UNUSED,
	DIGITAL_CHANNEL_5_UNUSED,
	DIGITAL_CHANNEL_6_UNUSED,
	DIGITAL_CHANNEL_7_UNUSED,
	DIGITAL_CHANNEL_8_UNUSED
}DIGITAL_IO_CHANNEL_TYPE;

typedef enum 
{
	RELAY_CHANNEL_1_COMPRESSOR_RELAY = 1,
	RELAY_CHANNEL_2_UNUSED,
	RELAY_CHANNEL_3_UNUSED,
	RELAY_CHANNEL_4_UNUSED,
	RELAY_CHANNEL_5_UNUSED,
	RELAY_CHANNEL_6_UNUSED,
	RELAY_CHANNEL_7_UNUSED,
	RELAY_CHANNEL_8_UNUSED,
}RELAY_CHANNEL_TYPE;

class Robot2012 : public IterativeRobot
{
	Jaguar jaguarFrontLeft;
	Jaguar jaguarFrontRight;
	Jaguar jaguarRearLeft;
	Jaguar jaguarRearRight;
	RobotDrive myRobot; // robot drive system
	Jaguar jaguarElevatorBottom1;
	Jaguar jaguarElevatorBottom2;
	Jaguar jaguarElevatorTop;
	Victor victorDumperRoller;
	Counter counterDumperRoller;
	Solenoid solenoidCatapultPusher;
	Solenoid solenoidCatapultPuller;
	Solenoid solenoidBridgeRamDown;
	Solenoid solenoidBridgeRamUp;
	Solenoid solenoidStingerExtend;
	Solenoid solenoidStingerRetract;
	Solenoid solenoidCatapultLatchExtend;
	Solenoid solenoidCatapultLatchRetract;
	Joystick stickRightDrive; // only joystick
	Joystick stickLeftDrive;
	Joystick stickShooter;
	KinectStick kinectLeft;
	KinectStick kinectRight;
	Gyro gyroscope;
	GyroControlledTurning rotationControl;
	PIDController rotationPID;
	AnalogRangeFinder rangeFinder;
	SonarControlledDriving sonarDriveControl;
	PIDController rangePID;
	Compressor compressor;

	Timer timeInState;
	Timer timeSinceBoot;
	Timer autonomousStateTimer;
	
	DriverStation *driverStation;
	DriverStationLCD *driverStationLCD;
		
		// Local variables to count the number of periodic loops performed
	
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
	
	bool cameraInitialized;
	float angleToTurn;
	float angleAtImage;
	float distanceToTarget;
	bool targetLocked;

	typedef enum 
	{
		AUTONOMOUS_LINING_UP_SHOT,
		AUTONOMOUS_FIRING_SHOT,
		AUTONOMOUS_SHOT_FIRED,
		AUTONOMOUS_REARMING_SHOT,
		AUTONOMOUS_RELOADING,
		AUTONOMOUS_HITTING_BRIDGE,
		AUTONOMOUS_DONE,
		
	}AUTONOMOUS_STATE;
	
	AUTONOMOUS_STATE autonomousState;
	typedef enum
	{
		AUTONOMOUS_MODE_TWO_BALL_AND_TIP,
		AUTONOMOUS_MODE_TWO_BALL,
		AUTONOMOUS_MODE_FOUR_BALL,
		AUTONOMOUS_MODE_SIX_BALL,
		AUTONOMOUS_MODE_FEED,
		AUTONOMOUS_MODE_FENDER_SHOTS,
		AUTONOMOUS_MODE_THREE_BALL
	}AUTONOMOUS_MODE_SELECT;
		
	AUTONOMOUS_MODE_SELECT autonomousMode;
	
	typedef enum
	{
		CATAPULT_COCKING,
		CATAPULT_WAITING_LATCH,
		CATAPULT_READY,
		CATAPULT_FIRING
	}CATAPULT_STATE;
	
	CATAPULT_STATE catapultState;
public:

	Robot2012(void):
		jaguarFrontLeft(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_4_JAGUAR_FRONT_LEFT),
		jaguarFrontRight(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_3_JAGUAR_FRONT_RIGHT),
		jaguarRearLeft(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_2_JAGUAR_REAR_LEFT),
		jaguarRearRight(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_1_JAGUAR_REAR_RIGHT),
		myRobot(&jaguarFrontLeft, &jaguarRearLeft, &jaguarFrontRight, &jaguarRearRight),
		jaguarElevatorBottom1(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_5_JAGUAR_ELEVATOR_BOTTOM_1),
		jaguarElevatorBottom2(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_6_JAGUAR_ELEVATOR_BOTTOM_2),
		jaguarElevatorTop(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_7_JAGUAR_ELEVATOR_TOP),
		victorDumperRoller(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_8_JAGUAR_DUMPER_ROLLER),
		counterDumperRoller(DIGITAL_OUTPUT_CHANNEL, DIGITAL_CHANNEL_2_DUMPER_COUNTER),
		solenoidCatapultPusher(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_1_CATAPULT_PUSHER),
		solenoidCatapultPuller(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_2_CATAPULT_PULLER),
		solenoidBridgeRamDown(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_3_BRIDGE_RAM_DOWN),
		solenoidBridgeRamUp(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_4_BRIDGE_RAM_UP),
		solenoidStingerExtend(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_6_STINGER_EXTEND),
		solenoidStingerRetract(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_5_STINGER_RETRACT),
		solenoidCatapultLatchExtend(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_7_CATAPULT_LATCH_EXTEND),
		solenoidCatapultLatchRetract(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_8_CATAPULT_LATCH_RETRACT),
		stickRightDrive(1),
		stickLeftDrive(2),
		stickShooter(3),
		kinectLeft(1),
		kinectRight(2),
		gyroscope(ANALOG_OUTPUT_CHANNEL, ANALOG_CHANNEL_1_GYROSCOPE),
		rotationControl(&myRobot),
		rotationPID(ROTATION_PID_PROPORTION, ROTATION_PID_INTEGRAL, ROTATION_PID_DERIVATIVE, &gyroscope, &rotationControl),
		rangeFinder(ANALOG_OUTPUT_CHANNEL, ANALOG_CHANNEL_2_RANGE_FINDER),
		sonarDriveControl(&myRobot),
		rangePID(RANGE_PID_PROPORTION,RANGE_PID_INTEGRAL,RANGE_PID_DERIVATIVE, &rangeFinder, &sonarDriveControl),
		compressor(DIGITAL_CHANNEL_1_INPUT_COMPRESSOR_SWITCH, RELAY_CHANNEL_1_COMPRESSOR_RELAY),
		timeInState(),
		timeSinceBoot()
	{
		printf("Robot2012 Constructor Started\n");
		cameraInitialized = false;
		// Acquire the Driver Station object
		driverStation = DriverStation::GetInstance();
		driverStationLCD = DriverStationLCD::GetInstance();
		myRobot.SetInvertedMotor(myRobot.kRearLeftMotor, true);
		myRobot.SetInvertedMotor(myRobot.kRearRightMotor, true);
		myRobot.SetInvertedMotor(myRobot.kFrontLeftMotor, true);
		myRobot.SetInvertedMotor(myRobot.kFrontRightMotor, true);
		
		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;
		rotationPID.SetInputRange(ROTATION_PID_MIN_INPUT,ROTATION_PID_MAX_INPUT);
		rotationPID.SetOutputRange(ROTATION_PID_MIN_OUTPUT,ROTATION_PID_MAX_OUTPUT);
		rotationPID.SetTolerance(ROTATION_PID_TOLERENCE_FIRST);
		rotationPID.Disable();
		
		rangePID.SetInputRange(RANGE_PID_MIN_INPUT,RANGE_PID_MAX_INPUT);
		rangePID.SetOutputRange(RANGE_PID_MIN_OUTPUT, RANGE_PID_MAX_OUTPUT);
		rangePID.SetSetpoint(RANGE_PID_SETPOINT);
		rangePID.SetTolerance(RANGE_PID_TOLERENCE);
		rangePID.Disable();
		
				
		
		printf("Robot2012 Constructor Completed\n");
	}
	
	
	/********************************** Init Routines *************************************/

	void RobotInit(void) 
	{
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		

		catapultState = CATAPULT_READY;
		autonomousMode = AUTONOMOUS_MODE_TWO_BALL_AND_TIP;
		
		timeSinceBoot.Start();
		myRobot.SetExpiration(1.0);
		printf("RobotInit() completed.\n");
	}
	
	void DisabledInit(void) 
	{
		m_autoPeriodicLoops = 0;				
		m_telePeriodicLoops = 0;				
		m_disabledPeriodicLoops = 0;			
		timeInState.Reset();
		timeInState.Start();
		compressor.Stop();
		myRobot.SetSafetyEnabled(false);
		ManageCatapult(false,false);

		autonomousMode = AUTONOMOUS_MODE_TWO_BALL_AND_TIP;
		
	}

	void AutonomousInit(void) 
	{
		m_autoPeriodicLoops = 0;				
		m_telePeriodicLoops = 0;				
		m_disabledPeriodicLoops = 0;			
		timeInState.Reset();
		timeInState.Start();
		compressor.Start();
		autonomousState = AUTONOMOUS_LINING_UP_SHOT;
		autonomousStateTimer.Reset();
		autonomousStateTimer.Start();
		myRobot.SetSafetyEnabled(true);
	}

	void TeleopInit(void) 
	{
		m_autoPeriodicLoops = 0;				
		m_telePeriodicLoops = 0;				
		m_disabledPeriodicLoops = 0;			
		timeInState.Reset();
		timeInState.Start();
		compressor.Start();
		myRobot.SetSafetyEnabled(true);
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  
	{
		m_disabledPeriodicLoops++;
		CameraInitialize();
		AxisCamera &camera = AxisCamera::GetInstance("10.24.74.11");
		if (camera.IsFreshImage() == true)
		{
			ColorImage *colorImage = new ColorImage(IMAQ_IMAGE_HSL);
			if (colorImage == (void *)0)
				return;
			camera.GetImage(colorImage);
			//TODO Print to the LCD as well
			if (DetermineTargetPosition(colorImage) == 0)
			{
				if ((angleToTurn > (ANGLE_POSITION_BASE - DISTANCE_POSITION_TOLERANCE)) && 
					(angleToTurn < (ANGLE_POSITION_BASE + DISTANCE_POSITION_TOLERANCE)) &&
					(distanceToTarget > (DISTANCE_POSITION_BASE - DISTANCE_POSITION_TOLERANCE)) && 
					(distanceToTarget < (DISTANCE_POSITION_BASE + DISTANCE_POSITION_TOLERANCE)))
					
				{
					SetRIOUserLED(1);
					//driverStationLCD->PrintfLine((DriverStationLCD::Line) 0, "Camera is ON target");
				}
				else
				{
					SetRIOUserLED(0);
					//driverStationLCD->PrintfLine((DriverStationLCD::Line) 0, "Camera is OFF target");
				}
			}
			else
			{
				SetRIOUserLED(0);
				//driverStationLCD->PrintfLine((DriverStationLCD::Line) 0, "Camera is OFF target");
			}
			delete colorImage;
		}
		
		static Timer button_combo_timer;
		if (BUTTON_COMBO_SWITCH_AUTONOMOUS() == true)
		{
			button_combo_timer.Start();
		}
		else
		{
			button_combo_timer.Reset();
			button_combo_timer.Stop();
		}
		
		if (button_combo_timer.Get() > 2.00)
		{
			button_combo_timer.Reset();
			switch (autonomousMode)
			{
			case AUTONOMOUS_MODE_TWO_BALL_AND_TIP:
				autonomousMode = AUTONOMOUS_MODE_TWO_BALL;
				break;
			case AUTONOMOUS_MODE_TWO_BALL:
				autonomousMode = AUTONOMOUS_MODE_THREE_BALL;
				break;
			case AUTONOMOUS_MODE_THREE_BALL:
				autonomousMode = AUTONOMOUS_MODE_FOUR_BALL;
				break;
			case AUTONOMOUS_MODE_FOUR_BALL:
//				autonomousMode = AUTONOMOUS_MODE_SIX_BALL;
//				break;
//			case AUTONOMOUS_MODE_SIX_BALL:
				autonomousMode = AUTONOMOUS_MODE_FEED;
				break;
			case AUTONOMOUS_MODE_FEED:
				autonomousMode = AUTONOMOUS_MODE_FENDER_SHOTS;
				break;
			default:
			case AUTONOMOUS_MODE_FENDER_SHOTS:
				autonomousMode = AUTONOMOUS_MODE_TWO_BALL_AND_TIP;
				break;
			}
		}
		
	
		
		//TODO look for joystick buttons instead of this switch
		switch (autonomousMode)
		{
		default:
		case AUTONOMOUS_MODE_TWO_BALL_AND_TIP:
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Using 2 ball and tip bridge auton");
			break;
		case AUTONOMOUS_MODE_FOUR_BALL:
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Using 4 ball autonomous");
			break;
		case AUTONOMOUS_MODE_SIX_BALL:
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Using 6 ball autonomous");
			break;
		case AUTONOMOUS_MODE_TWO_BALL:
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Using 2 ball NO BRIDGE TIP");
			break;
		case AUTONOMOUS_MODE_THREE_BALL:
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Using 3 ball NO BRIDGE TIP");
			break;
		case AUTONOMOUS_MODE_FEED:
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Autonomous Feeding balls");
			break;
		case AUTONOMOUS_MODE_FENDER_SHOTS:
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shooting from the fender");
			break;
		}
		driverStationLCD->UpdateLCD();
	}

	bool CatapultAutonomous(bool hit_bridge, int number_of_shots)
	{
		bool running_auton = false;
		static int shots_fired;
		if (m_autoPeriodicLoops < 2)
		{
			shots_fired = 0;
		}
		bool use_slow_roller = ((number_of_shots > 2) && (shots_fired < number_of_shots));
		if (shots_fired < number_of_shots)
		{
			running_auton = true;
			ManageAppendages(false,false);
			PositionForTarget(LINING_UP_IN_AUTONOMOUS);
			
			switch (autonomousState)
			{
			default:
			case AUTONOMOUS_FIRING_SHOT:
				ManageCatapult(true, true);
				shots_fired++;
			case AUTONOMOUS_SHOT_FIRED:
				autonomousState = AUTONOMOUS_SHOT_FIRED;
				ManageCatapult(false, false);
				ManageElevator(false,false,false,false,false,use_slow_roller);
				if (catapultState == CATAPULT_COCKING)
				{
					autonomousState = AUTONOMOUS_REARMING_SHOT;
				}
				break;
			//Cumulative time: 0.01
			case AUTONOMOUS_REARMING_SHOT:
				ManageAppendages(false,false);

				//For the first shot, don't pickup from the floor because it could jam the loading of ball 2
				if (shots_fired < 2)
				{
					ManageElevator(false,false,false,false,false,false);
				}
				else
				{
					ManageElevator(true,false,false,false,false,false);
				}
				PositionForTarget(LINING_UP_IN_AUTONOMOUS);
				ManageCatapult(false, false);
				if (catapultState != CATAPULT_COCKING)
				{
					autonomousStateTimer.Reset();
					autonomousState = AUTONOMOUS_RELOADING;
				}
				break;
			//Cumulative time: 1.0
			//Load a ball after 1.0 seconds and before 4.4 seconds (2.9 ideal)
			case AUTONOMOUS_RELOADING:
				ManageAppendages(false,false);
				if (autonomousStateTimer.Get() > SLOW_PICKUP_DURING_RELOAD_START)
				{
					ManageElevator(false,false,false,true,false,use_slow_roller);
				}
				else
				{
					ManageElevator(false,false,false,true,false,false);
				}
				PositionForTarget(LINING_UP_IN_AUTONOMOUS);
				ManageCatapult(false, false);
				if (autonomousStateTimer.Get() > CATAPULT_RELOAD_TIME_FOUR_BALL)
				{
					ManageCatapult(false, false);
					autonomousStateTimer.Reset();
					autonomousState = AUTONOMOUS_FIRING_SHOT;
				}
				break;
			}
		}
		else if (hit_bridge == true)
		{
			ManageAppendages(true,false);
			PositionForTarget(false);
			ManageCatapult(false, false);
			
			autonomousState = AUTONOMOUS_HITTING_BRIDGE;
			ManageElevator(true,false,false,false,false,false);
			if (autonomousStateTimer.Get() < AUTONOMOUS_BACKUP_TIME_WAIT)
			{
				myRobot.TankDrive(0.0,0.0);
				running_auton = true;
			}
			else if (autonomousStateTimer.Get() < AUTONOMOUS_BACKUP_TIME_FAST)
			{
				myRobot.TankDrive(AUTONOMOUS_BACKUP_SPEED_FAST,AUTONOMOUS_BACKUP_SPEED_FAST);
				running_auton = true;
			}
			else if (autonomousStateTimer.Get() < AUTONOMOUS_BACKUP_TIME_SLOW)
			{
				myRobot.TankDrive(AUTONOMOUS_BACKUP_SPEED_SLOW,AUTONOMOUS_BACKUP_SPEED_SLOW);
				running_auton = true;
			}
			else
			{
				myRobot.TankDrive(0.0,0.0);
			}
		}
		
		return running_auton;
	}
	void AutonomousPeriodic(void) 
	{
		m_autoPeriodicLoops++;
		CameraInitialize();
		if (autonomousMode == AUTONOMOUS_MODE_TWO_BALL_AND_TIP)
		{
			if (CatapultAutonomous(true,2))
				return;
		}
		else if(autonomousMode == AUTONOMOUS_MODE_TWO_BALL)
		{
			if (CatapultAutonomous(false,2))
				return;
		}
		else if (autonomousMode == AUTONOMOUS_MODE_THREE_BALL)
		{
			if (CatapultAutonomous(false, 3))
				return;
		}
		else if (autonomousMode == AUTONOMOUS_MODE_FOUR_BALL)
		{
			if (CatapultAutonomous(false,4))
				return;
		}
		else if (autonomousMode == AUTONOMOUS_MODE_SIX_BALL)
		{
			if (CatapultAutonomous(false, 6))
				return;
		}
		else if (autonomousMode == AUTONOMOUS_MODE_FEED)
		{
			autonomousStateTimer.Start();
			if (autonomousStateTimer.Get() > 7.0)
			{
				ManageElevator(false,true,false,true,false,false);
				return;
			}
			else if (autonomousStateTimer.Get() > 5.0)
			{
				ManageElevator(false,true,false,false,false,false);
				return;
			}
			else
			{
				ManageElevator(false,false,false,false,false,false);
			}
		}
		else if (autonomousMode == AUTONOMOUS_MODE_FENDER_SHOTS)
		{
			autonomousStateTimer.Start();
			if (autonomousStateTimer.Get() < 3.5)
			{
				myRobot.TankDrive(AUTONOMOUS_FENDER_SHOT_FAST,AUTONOMOUS_FENDER_SHOT_FAST);
				ManageElevator(false,false,false,false,true,false);
			}
			else if (autonomousStateTimer.Get() < 4.0)
			{
				myRobot.TankDrive(AUTONOMOUS_FENDER_SHOT_SLOW,AUTONOMOUS_FENDER_SHOT_SLOW);
				ManageElevator(false,false,true,false,true,false);
			}
			else if (autonomousStateTimer.Get() < 8.0)
			{
				myRobot.TankDrive(0.0,0.0);
				ManageElevator(false,true,true,false,true,false);
			}
			else if (autonomousStateTimer.Get() < 12.0)
			{
				myRobot.TankDrive(-AUTONOMOUS_FENDER_SHOT_FAST,-AUTONOMOUS_FENDER_SHOT_FAST);
				ManageElevator(false,true,true,false,true,false);
			}
		}
		
		//If we didn't return above, allow the kinect to function
		if (KINECT_CONTROL_ENABLED() == true)
		{ 
			myRobot.TankDrive(kinectLeft.GetY() * 0.7, kinectRight.GetY() * 0.7);
			//add code to bind each kinectStick button to each action we want to be able to do in autonomous
			//ManageAppendages(KINECT_RIGHT_LEG_BACK(),false);
			BRIDGE_RAM_EXTENDED(KINECT_BRIDGE_RAM_EXTEND());
			ManageElevator((KINECT_ELEVATORS_UP() || KINECT_BRIDGE_RAM_EXTEND()),KINECT_ELEVATORS_DOWN(),
					KINECT_ELEVATORS_UP(),KINECT_ELEVATORS_DOWN(), 
					KINECT_ELEVATORS_UP(),false);
			PositionForTarget(false);
			ManageCatapult(false, false);
		}
	}

	
	void TeleopPeriodic(void) 
	{
		// increment the number of teleop periodic loops completed
		m_telePeriodicLoops++;
		
		if (!BUTTON_CAMERA_ALIGN_SHOT_BUTTON())
		{
			DISABLE_PID(rotationPID);
			DISABLE_PID(rangePID);
			myRobot.TankDrive(stickLeftDrive,stickRightDrive);	
			myRobot.SetSafetyEnabled(true);
		}
		CameraInitialize();

		
		ManageAppendages(BUTTON_BRIDGE_RAM_TOGGLE(),BUTTON_STINGER_TOGGLE());
		ManageElevator(BUTTON_ELEVATOR_BOTTOM_UP(), BUTTON_ELEVATOR_BOTTOM_DOWN(), 
						BUTTON_ELEVATOR_TOP_UP(), BUTTON_ELEVATOR_TOP_DOWN(), 
						BUTTON_DUMPER_ROLLER(), false);
		ManageCatapult(BUTTON_CATAPULT_SHOOT(), BUTTON_CATAPULT_FORCE_SHOOT());

		PositionForTarget(BUTTON_CAMERA_ALIGN_SHOT_BUTTON());

	}


/********************************** Continuous Routines *************************************/
//	void DisabledContinuous(void) 
//	{
//		//CameraInitialize();
//	}
//
//	void AutonomousContinuous(void)	
//	{
//		//AxisCamera &camera = AxisCamera::GetInstance("10.24.74.11");
//		//CameraInitialize();
//	}
//	void TeleopContinuous(void) 
//	{
//	}

	//returns 0 if a particle is found
	int DetermineTargetPosition(ColorImage *colorImage)
	{
		int returnVal = -1;
		BinaryImage *binaryImage;
		Image *imaqImage;
		if ((colorImage == (void *) 0) || (colorImage->GetWidth() == 0) || (colorImage->GetHeight() == 0))
		{
			return returnVal;
		}
		if (BUTTON_CAMERA_TAKE_DEBUG_PICTURES())
		{
			colorImage->Write("capturedImage.jpg");
		}
		
		//binaryImage = colorImage->ThresholdHSV(0, 255, 0, 255, 255, 0);
		//binaryImage = colorImage->ThresholdHSV(56, 125, 55, 255, 255, 150);
		
		binaryImage = colorImage->ThresholdHSL(90, 115,30, 255, 70, 255);
		
		if ((binaryImage == (void *) 0) || (binaryImage->GetWidth() == 0) || (binaryImage->GetHeight() == 0))
			return returnVal;
		
		imaqImage = binaryImage->GetImaqImage();
		
		if (imaqImage == (void *) 0)
			return returnVal;
		
		if (BUTTON_CAMERA_TAKE_DEBUG_PICTURES())
		{
			binaryImage->Write("afterCLRThreshold.bmp");
			IVA_ProcessImage(imaqImage, binaryImage);
		}	
		else
		{
			IVA_ProcessImage(imaqImage, (ImageBase *)0);
		}
		vector<ParticleAnalysisReport> *reports = binaryImage->GetOrderedParticleAnalysisReports();
		ParticleAnalysisReport *topParticlePtr = NULL;
		
		for (int particleIndex = 0; particleIndex < reports->size(); particleIndex++)
		{
			ParticleAnalysisReport &thisReport = reports->at(particleIndex);
			//TODO Validate the particle so we don't end up targeting a spec on the wall? (particle filter gets rid fo really small stuff, but we may want this if the arena is noisy)
			if ((!topParticlePtr) || (thisReport.center_mass_y  < topParticlePtr->center_mass_y)  /*&& thisReport.particleQuality > WHATT!!!!*/)
			{
				topParticlePtr = &thisReport;
			}
		}
		if (topParticlePtr != NULL)
		{
			returnVal = 0; //indicates success
			int centerOfMassX = topParticlePtr->center_mass_x;
			
			angleToTurn = (centerOfMassX - CENTER_OF_IMAGE) * HORIZONTAL_DEGREES_PER_PIXEL;
			angleToTurn = ((-angleToTurn) + ROTATION_PID_SETPOINT_OFFSET);
			
			distanceToTarget = 12 * CALCULATE_DISTANCE(topParticlePtr->boundingRect.height);

			driverStationLCD->PrintfLine((DriverStationLCD::Line) 2, "Angle to turn: %.5f", -angleToTurn);
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Range to target: %.5f", distanceToTarget);
			driverStationLCD->UpdateLCD();

			#ifdef PID_TUNING
				static float tol_angle = ROTATION_PID_TOLERENCE;
				static float set_angle = ROTATION_PID_SETPOINT_OFFSET;
				float p = rotationPID.GetP();
				float i = rotationPID.GetI();
				float d = rotationPID.GetD();
				rotationPID.SetPID(p,i,d);
				
				rotationPID.SetTolerance(tol_angle);
				rotationPID.SetSetpoint(angleToTurn);
				
				static float tol_range = RANGE_PID_TOLERENCE;
				static float set_range = RANGE_PID_SETPOINT;
				p = rangePID.GetP();
				i = rangePID.GetI();
				d = rangePID.GetD();
				rangePID.SetPID(p,i,d);
				rangePID.SetTolerance(tol_range);
				rangePID.SetSetpoint(set_range);
			#else
			rotationPID.SetSetpoint(angleToTurn);
			#endif
			
			
		}	
		else
		{	
			jaguarFrontLeft.Set(0.0);
			jaguarFrontRight.Set(0.0);
			jaguarRearLeft.Set(0.0);
			jaguarRearRight.Set(0.0);
		}
		delete reports;
		delete binaryImage;

		return returnVal;
	}
	

	void ManageAppendages(bool extend_bridge_ram, bool extend_stinger)
	{
		typedef enum
		{
			APPENDAGE_IDLE,
			APPENDAGE_BRIDGE_RAM_EXTENDED,
			APPENDAGE_BRIDGE_RAM_RETRACTING,
			APPENDAGE_STINGER_EXTENDED,
			APPENDAGE_STINGER_RETRACTING
		}APPENDAGE_STATE;
		
		static int retracting_appendage_counter = 0;
		static APPENDAGE_STATE appendage_state = APPENDAGE_IDLE;
		static bool trigger_released = false;
		switch(appendage_state)
		{
		default:
		case APPENDAGE_IDLE:
			BRIDGE_RAM_EXTENDED(false);
			STINGER_EXTENDED(false);
			if (!extend_bridge_ram && !extend_stinger)
			{
				trigger_released = true;
			}
			
			if(trigger_released && extend_bridge_ram)
			{
				trigger_released = false;
				BRIDGE_RAM_EXTENDED(true);
				appendage_state = APPENDAGE_BRIDGE_RAM_EXTENDED;
			}
			else if (trigger_released && extend_stinger)
			{
				trigger_released = false;
				STINGER_EXTENDED(true);
				appendage_state = APPENDAGE_STINGER_EXTENDED;
			}
			break;
		case APPENDAGE_BRIDGE_RAM_EXTENDED:
			BRIDGE_RAM_EXTENDED(true);
			STINGER_EXTENDED(false);

			if (!extend_bridge_ram)
			{
				trigger_released = true;
			}
			if (trigger_released && extend_bridge_ram)
			{
				trigger_released = false;
				BRIDGE_RAM_EXTENDED(false);
				STINGER_EXTENDED(false);
				retracting_appendage_counter = 0;
				appendage_state = APPENDAGE_STINGER_RETRACTING;
			}
			break;
		case APPENDAGE_BRIDGE_RAM_RETRACTING:
			BRIDGE_RAM_EXTENDED(false);
			STINGER_EXTENDED(false);
			if (!extend_bridge_ram)
			{
				trigger_released = true;
			}
			
			retracting_appendage_counter++;
			if (retracting_appendage_counter > 10)
			{
				appendage_state = APPENDAGE_IDLE;
			}
			break;
		case APPENDAGE_STINGER_EXTENDED:

			BRIDGE_RAM_EXTENDED(false);
			STINGER_EXTENDED(true);

			if (!extend_stinger)
			{
				trigger_released = true;
			}
			if (trigger_released && extend_stinger)
			{
				trigger_released = false;
				BRIDGE_RAM_EXTENDED(false);
				STINGER_EXTENDED(false);
				retracting_appendage_counter = 0;
				appendage_state = APPENDAGE_STINGER_RETRACTING;
			}
			break;
		case APPENDAGE_STINGER_RETRACTING:
			BRIDGE_RAM_EXTENDED(false);
			STINGER_EXTENDED(false);
			if (!extend_stinger)
			{
				trigger_released = true;
			}
			retracting_appendage_counter++;
			if (retracting_appendage_counter > 10)
			{
				appendage_state = APPENDAGE_IDLE;
			}
			break;
		}
	}

	void ManageElevator(bool bottom_up, bool bottom_down, bool top_up, bool top_down, bool dumper_roller, bool bottom_elevator_slow_up)
	{
		static Timer elevatorTimer;
		static float dumper_roller_power = 0.0;
		
#ifdef TEST_ELEVATOR_TIMING
		if (!(bottom_up || top_down))
		{
			elevatorTimer.Reset();
		}
#endif
		if (bottom_up)
		{
#ifdef TEST_ELEVATOR_TIMING
			//.75 second minimum
			elevatorTimer.Start();
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Bottom Timer: %f", elevatorTimer.Get());
#endif
			jaguarElevatorBottom1.Set(-ELEVATOR_SPEED_BOTTOM, BOTTOM_ROLLERS_SYNC_GROUP);
			jaguarElevatorBottom2.Set(-ELEVATOR_SPEED_BOTTOM, BOTTOM_ROLLERS_SYNC_GROUP);
		}
		else if (bottom_down)
		{
			jaguarElevatorBottom1.Set(ELEVATOR_SPEED_BOTTOM, BOTTOM_ROLLERS_SYNC_GROUP);
			jaguarElevatorBottom2.Set(ELEVATOR_SPEED_BOTTOM, BOTTOM_ROLLERS_SYNC_GROUP);
		}
		else if (bottom_elevator_slow_up)
		{
			jaguarElevatorBottom1.Set(-ELEVATOR_SPEED_BOTTOM_SLOW, BOTTOM_ROLLERS_SYNC_GROUP);
			jaguarElevatorBottom2.Set(-ELEVATOR_SPEED_BOTTOM_SLOW, BOTTOM_ROLLERS_SYNC_GROUP);
		}
		else
		{
			jaguarElevatorBottom1.Set(0.0);
			jaguarElevatorBottom2.Set(0.0);
		}
		
		if (top_up)
		{
			jaguarElevatorTop.Set(ELEVATOR_SPEED_TOP);
		}
		else if (top_down)
		{
#ifdef TEST_ELEVATOR_TIMING
			elevatorTimer.Start();
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Tm:%f spd%f", elevatorTimer.Get(), ELEVATOR_SPEED_TOP);
#endif
			jaguarElevatorTop.Set(-ELEVATOR_SPEED_TOP);
		}
		else
		{
			jaguarElevatorTop.Set(0.0);
		}
		
		float roller_rpm = GetRollerVelocity(dumper_roller);
		
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Roller RPM: %f", roller_rpm);
		driverStationLCD->UpdateLCD();
		
		if (dumper_roller)
		{
#ifdef DUMPER_ROLLER_RPM
			if (!BUTTON_CATAPULT_FORCE_SHOOT())
			{
				if (roller_rpm < DUMPER_ROLLER_RPM)
				{
					if (dumper_roller_power < (1 - DUMPER_ROLLER_VOLTAGE_RAMP_RATE))
					{
						dumper_roller_power += DUMPER_ROLLER_VOLTAGE_RAMP_RATE;
					}
					else
					{
						dumper_roller_power = 1.0F;
					}
				}
				else
				{
					if (dumper_roller_power > DUMPER_ROLLER_VOLTAGE_RAMP_RATE)
					{
						dumper_roller_power -= DUMPER_ROLLER_VOLTAGE_RAMP_RATE;
					}
					else
					{
						dumper_roller_power = 0.0F;
					}
				}
			}
			else
#endif
			{
				dumper_roller_power = DUMPER_ROLLER_POWER;
			}
			if (compressor.Enabled() == true)
			{
				compressor.Stop();
			}
		}
		else
		{
			dumper_roller_power = 0.0;			
			if (compressor.Enabled() == false)
			{
				compressor.Start();
			}
		}
		victorDumperRoller.Set(dumper_roller_power);
	}
	
	void ManageCatapult(bool catapult_shoot, bool force_shoot)
	{
		static bool button_released = true;
		static Timer state_timer;
		state_timer.Start();//Doesn't do anything unless it's not running
		
		if (!catapult_shoot)button_released = true;
		
		switch (catapultState)
		{
		case CATAPULT_COCKING:
			CATAPULT_PUSHER_EXTENDED(true);
			CATAPULT_LATCH_EXTENDED(false);
			if (state_timer.Get() >= CATAPULT_REARM_TIME)
			{
				state_timer.Reset();
				catapultState = CATAPULT_WAITING_LATCH;
			}
			break;
		case CATAPULT_WAITING_LATCH:
			CATAPULT_PUSHER_EXTENDED(false);
			CATAPULT_LATCH_EXTENDED(true);
			if (state_timer.Get() > CATAPULT_LATCH_TIME)
			{
				state_timer.Reset();
				catapultState = CATAPULT_READY;
			}
			break;
		case CATAPULT_READY:
			if (catapult_shoot && button_released)
			{
				if ((force_shoot == true) || (targetLocked == true)) 
				{
					CATAPULT_LATCH_EXTENDED(false);
					state_timer.Reset();
					catapultState = CATAPULT_FIRING;
				}
			}
			break;
		case CATAPULT_FIRING:
			CATAPULT_PUSHER_EXTENDED(false);
			CATAPULT_LATCH_EXTENDED(false);
			if (state_timer.Get() >= CATAPULT_FIRE_TIME)
			{
				state_timer.Reset();
				//CATAPULT_PUSHER_EXTENDED(true);
				catapultState = CATAPULT_COCKING;
			}
			break;
		}
	}

	void PositionForTarget(bool camera_align_shot)
	{
		typedef enum
		{
			TARGETING_IDLE,
			TARGETING_ROTATING,
			TARGETING_DRIVING_TO_DISTANCE,
			TARGETING_ROTATING_FINAL
		}TARGETING_STATE;
		
		static TARGETING_STATE state_targeting = TARGETING_IDLE;
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 4, "Angle Turned: %f", gyroscope.GetAngle());
		//driverStationLCD->PrintfLine((DriverStationLCD::Line) 4, "Range to target: %f", rangeFinder.GetRangeInches());
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 5, "Autoposition State: %d", state_targeting);
		driverStationLCD->UpdateLCD();
		
		static int on_target_count = 0;
		Timer imageRefreshTimer;
		AxisCamera &camera = AxisCamera::GetInstance("10.24.74.11");
		switch (state_targeting)
		{
		case TARGETING_IDLE:
			targetLocked = false;

			DISABLE_PID(rotationPID);
			DISABLE_PID(rangePID);
			if (camera.IsFreshImage())
			{
				if (camera_align_shot == true)
				{
					ColorImage *colorImage = new ColorImage(IMAQ_IMAGE_HSL);
					camera.GetImage(colorImage);
					imageRefreshTimer.Reset();
					imageRefreshTimer.Start();
					gyroscope.Reset();
					if (DetermineTargetPosition(colorImage) == 0)
					{
						rotationPID.SetTolerance(ROTATION_PID_TOLERENCE_FIRST);
						rotationPID.Enable();
						state_targeting = TARGETING_ROTATING;
						on_target_count = 0;
					}
					delete colorImage;
				}
			}
			break;
		case TARGETING_ROTATING:
			if (camera_align_shot == false)
			{
				DISABLE_PID(rotationPID);
				state_targeting = TARGETING_IDLE;
				on_target_count = 0;
			}
			else if (rotationPID.OnTarget() && (on_target_count > 3))
			{
				on_target_count = 0;
#ifdef DISABLE_RANGE_FINDER
				state_targeting = TARGETING_ROTATING_FINAL;
#else
				
				#ifdef DISABLE_RANGE_ADJUST_IN_AUTONOMOUS
				if (m_autoPeriodicLoops != 0)
				{
					state_targeting = TARGETING_ROTATING_FINAL;
				}
				else
				#endif
				{
					DISABLE_PID(rotationPID);
					rangePID.Enable();
					state_targeting = TARGETING_DRIVING_TO_DISTANCE;
				}
#endif
			}
			else if (rotationPID.OnTarget())
			{
				rotationPID.Enable();
				on_target_count++;
			}
			else
			{
				rotationPID.Enable();
				on_target_count = 0;
			}
			break;
			//TODO need to put in a small delay for autonomous
		case TARGETING_DRIVING_TO_DISTANCE:
			if (camera_align_shot == false)
			{
				DISABLE_PID(rangePID);
				state_targeting = TARGETING_IDLE;
				on_target_count = 0;
			}
			else if (rangePID.OnTarget() && (on_target_count > 8))
			{
				DISABLE_PID(rangePID);
				state_targeting = TARGETING_ROTATING_FINAL;
				rotationPID.SetTolerance(ROTATION_PID_TOLERENCE_LAST);
				on_target_count = 0;
			}
			else if (rangePID.OnTarget())
			{
				rangePID.Enable();
				on_target_count++;
			}
			else
			{
				rangePID.Enable();
				on_target_count = 0;
			}
			break;
		case TARGETING_ROTATING_FINAL:
			
			if (camera.IsFreshImage() && camera_align_shot && (imageRefreshTimer.HasPeriodPassed(1.0)))
			{
				ColorImage *colorImage = new ColorImage(IMAQ_IMAGE_HSL);
				camera.GetImage(colorImage);
				imageRefreshTimer.Reset();
				imageRefreshTimer.Start();
				if (DetermineTargetPosition(colorImage) == 0)
				{
					gyroscope.Reset();
				}
				delete colorImage;
			}
			DISABLE_PID(rangePID);
			if (camera_align_shot == false)
			{
				DISABLE_PID(rotationPID);
				state_targeting = TARGETING_IDLE;
				targetLocked = false;
				on_target_count = 0;
			}
			else if (rotationPID.OnTarget() && (on_target_count > 6))
			{
				targetLocked = true;
				rotationPID.Enable();
			}
			else if (rotationPID.OnTarget())
			{
				rotationPID.Enable();
				on_target_count++;
			}
			else
			{
				rotationPID.Enable();
				targetLocked = false;
				on_target_count = 0;
			}
			break;
		}
	}
	

	void CameraInitialize(void)
	{
		if (cameraInitialized == false)
		{
			AxisCamera &camera = AxisCamera::GetInstance("10.24.74.11");
			if (&camera != (void *)0)
			{
				camera.WriteBrightness(CAMERA_BRIGHTNESS_LEVEL);
				camera.WriteColorLevel(CAMERA_COLOR_LEVEL);
				camera.WriteMaxFPS(CAMERA_MAX_FPS);
				//camera.WriteResolution(camera.kResolution_320x240);
				//camera.WriteWhiteBalance(camera.kWhiteBalance_FixedFlourescent1);
			}
		}
	}
	
	void FlipSolenoids(bool isFirstEna, Solenoid *sol1, Solenoid *sol2)
	{
		if ((sol1 == NULL) || (sol2 == NULL))
			return;
		
		if (isFirstEna)
		{
			sol2->Set(false);
			sol1->Set(true);
		}
		else
		{
			sol1->Set(false);
			sol2->Set(true);
		}
	}
	
	float GetRollerVelocity(bool roller_on)
	{
		static UINT32 prev_time = 0;
		static float filtered_rpm = 0;
		float return_rpm = 0;
		
		if (prev_time == 0)
		{
			prev_time = GetFPGATime();
			counterDumperRoller.Start();
			return return_rpm;
		}
		
		if ((filtered_rpm > 100) || (roller_on == true))
		{
			UINT32 time_difference = GetFPGATime() - prev_time;
			INT32 count = counterDumperRoller.Get();
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Roller count: %d", count);
			//Filter the incoming speed against previous speed to get a 
			float current_rpm = ((float)count) / ((float)(((float)DUMPER_ROLLER_COUNTS_PER_REVOLUTION/(float)FPGA_TIME_TO_MINUTES_FACTOR) * time_difference)); 
			filtered_rpm = (current_rpm * (1 - DUMPER_ROLLER_FILTER_CONSTANT)) + (filtered_rpm * DUMPER_ROLLER_FILTER_CONSTANT);
			return_rpm = filtered_rpm;
		}
		
		counterDumperRoller.Reset();
		prev_time = GetFPGATime();
		
		return return_rpm;
	}
};


START_ROBOT_CLASS(Robot2012);
