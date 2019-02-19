/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//for universal variables (i.e. ports, sensor/actuator configs)
public class RobotMap {
	public static final double ENCODER_CONVERSION_CONSTANT = 2144.660585;
	//climber motors
	public static final int leftClimber_motor_port = 0;
	public static final int rightClimber_motor_port = 0;

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
		public static final double kP = 0.023;
		public static final double kI = 0.0026;
		public static final double kD = 0.25;
		public static final double kF = 0.1;
		public static final double kToleranceDegrees = 1.0f;
	}

	public class ArmAssemblyMap {
		//Motors
		public static final int topFinger_motor_port = 0;
		public static final int bottomFinger_motor_port = 0;
		public static final int wrist_motor_port = 0;
		public static final int armMaster_motor_port = 0;
		public static final int armFollower_motor_port = 0;
		public static final int leftIntakeEnd_motor_port = 0;
		public static final int leftIntakeArm_motor_port = 0;
		public static final int rightIntakeEnd_motor_port = 0;
		public static final int rightIntakeArm_motor_port = 0;

		//Potentiometers
		public static final int topFinger_pot_port = 0;
		public static final int bottomFinger_pot_port = 0;
		public static final int wrist_pot_port = 0;
		public static final int arm_pot_port = 0;
		public static final int leftIntakeArm_pot_port = 0;
		public static final int rightIntakeArm_pot_port = 0;
	}

	public enum Joint {
		kARM,
		kLEFT_INTAKE,
		kRIGHT_INTAKE,
		kTOP_FINGER,
		kBOTTOM_FINGER,
		kWRIST
	}

	public enum ArmPosition {
		kHATCH,
		kBALL_PICKUP,
		kSHOOT_LOW,
		kSHOOT_MID
	}
}