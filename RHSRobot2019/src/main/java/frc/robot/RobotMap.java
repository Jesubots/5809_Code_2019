/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//for universal variables (i.e. ports, sensor/actuator configs)
public class RobotMap {
	//constants
	public static final double ENCODER_CONVERSION_CONSTANT = 2144.660585;
	public static final double ARM_WRIST_RATIO = 1f;
	public static final double MAXIMUM_INTAKE_ANGLE = 90f;
	public static final double climberTimeout = 1f;
	//climber
	public static final int leftClimber_motor_port = 1;
	public static final int rightClimber_motor_port = 6;
	//solenoid
	public static final int leftPunchForward_sol_port = 5;
	public static final int rightPunchForward_sol_port = 3;
	public static final int brakeForward_sol_port = 1;
	public static final int leftPunchReverse_sol_port = 4;
	public static final int rightPunchReverse_sol_port = 2;
	public static final int brakeReverse_sol_port = 0;
	//drivetrain
	public static final int backLeft_port = 0;
	public static final int frontLeft_port = 4;
	public static final int backRight_port = 7;
	public static final int frontRight_port = 2;
	//AN
	public static final int rightPistonMag_port = 0;
	public static final int leftPistonMag_port = 1;
	//navx runs AN ports 4-7, referred to as 0-3
	public static final int frontUltrasonic_port = 4;
	public static final int backUltrasonic_port = 5;
	//DIO
	public static final int limitSwitch_port = 3;


	//PID values for encoders on drive train
	public class PolarPIDMap { 
		public static final double kP = 0.023;
		public static final double kI = 0.0026;
		public static final double kD = 0.25;
		public static final double kF = 0.1;
		public static final double kToleranceDegrees = 1.0f;
	}

	//PID values for potentiometers on joints
	public class PotPIDMap { 
		public static final double kP = 0.1;
		public static final double kI = 0.001;
		public static final double kD = 1.0;
		public static final double kF = 0.0001;
		public static final double kToleranceDegrees = 1.0f;
	}

	//PID values for Encoders on joints
	public class EncoderJointPIDMap { 
		public static final double kP = 0.01;
		public static final double kI = 0.001;
		public static final double kD = 0.5;
		public static final double kF = 0.1;
		public static final double kToleranceDegrees = 1.0f;
	}

	public class PivotTurnPIDMap {
		public static final double kP = 0.023;
		public static final double kI = 0.0026;
		public static final double kD = 0.25;
		public static final double kF = 0.1;
		public static final double kToleranceDegrees = 1.0f;
	}

	public class CameraPIDMap {
		public static final double kP = 0.023;
		public static final double kI = 0.0026;
		public static final double kD = 0.25;
		public static final double kF = 0.1;
		public static final double kToleranceDegrees = 1.0f;
	}

	public class ArmAssemblyMap {
		//Motors
		public static final int topFinger_motor_port = 7;
		public static final int bottomFinger_motor_port = 3;
		public static final int wrist_motor_port = 6;
		public static final int armFront_motor_port = 1;
		public static final int armBack_motor_port = 3;
		public static final int leftIntakeEnd_motor_port = 5;
		public static final int leftIntakeArm_motor_port = 2;
		public static final int rightIntakeEnd_motor_port = 0;
		public static final int rightIntakeArm_motor_port = 4;

		//Potentiometers
		public static final int topFinger_pot_port = 0;
		public static final int bottomFinger_pot_port = 1;
		public static final int rightIntake_pot_port = 2;
		public static final int leftIntake_pot_port = 3;
	}

	public enum Joint {
		kARM,
		kINTAKE,
		kTOP_FINGER,
		kBOTTOM_FINGER,
		kWRIST
	}

	public enum ArmPosition {
		kHATCH,
		kBALL_PICKUP,
		kSHOOT_LOW,
		kSHOOT_MID,
		kSHOOT_CARGO,
		kDEFAULT,
		kHOLDING
	}

	public enum BallTarget {
		kMID,
		kLOW,
		kCARGO
	}
}