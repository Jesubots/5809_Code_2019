/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ArmPosition;
import frc.robot.RobotMap.Joint;
import frc.robot.subsystems.PID.ArmPID;
import frc.robot.subsystems.PID.BackFingerPID;
import frc.robot.subsystems.PID.FrontFingerPID;
import frc.robot.subsystems.PID.LeftIntakePID;
import frc.robot.subsystems.PID.RightIntakePID;
import frc.robot.subsystems.PID.WristPID;

/**
 * Add your docs here.
 */
public class ArmAssembly extends Subsystem {
  public static VictorSPX backFinger_motor = new VictorSPX(RobotMap.ArmAssemblyMap.backFinger_motor_port);
  public static VictorSPX frontFinger_motor = new VictorSPX(RobotMap.ArmAssemblyMap.frontFinger_motor_port);
  public static WPI_TalonSRX wrist_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.wrist_motor_port);
  public static WPI_TalonSRX armFront_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.armFront_motor_port);
  public static WPI_TalonSRX armBack_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.armBack_motor_port);
  public static VictorSPX leftIntakeEnd_motor = new VictorSPX(RobotMap.ArmAssemblyMap.leftIntakeEnd_motor_port);
  public static VictorSPX leftIntakeArm_motor = new VictorSPX(RobotMap.ArmAssemblyMap.leftIntakeArm_motor_port);
  public static VictorSPX rightIntakeEnd_motor = new VictorSPX(RobotMap.ArmAssemblyMap.rightIntakeEnd_motor_port);
  public static VictorSPX rightIntakeArm_motor = new VictorSPX(RobotMap.ArmAssemblyMap.rightIntakeArm_motor_port);

  public Potentiometer backFinger_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.backFinger_pot_port, 3600, 0);
  public Potentiometer frontFinger_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.frontFinger_pot_port, 3600, 0);
  public Potentiometer leftIntake_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.leftIntake_pot_port, 3600, 0);
  public Potentiometer rightIntake_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.rightIntake_pot_port, 3600, 0);

  private ArmPosition armPosition = ArmPosition.DEFAULT;

  public static BackFingerPID backFingerPID = new BackFingerPID();
  public static FrontFingerPID frontFingerPID = new FrontFingerPID();
  public static RightIntakePID rightIntakePID = new RightIntakePID();
  public static LeftIntakePID leftIntakePID = new LeftIntakePID();
  public static ArmPID armPID = new ArmPID();
  public static WristPID wristPID = new WristPID();


  public double wristMult = .5;
  public double armMult = .5;
  public double intakeThreshold = 0;

  private double backFingerOffset;
  private double frontFingerOffset;
  private double leftIntakeOffset;
  private double rightIntakeOffset;

  public void setArmMult(double mult){
    armMult -= mult;
    
    if(armMult <= 0){
      armMult = 1.0;
    }

    //System.out.println"Arm Multiplier" + armMult);
  }

  public void setWristMult(double mult){
    wristMult -= mult;
    
    if(wristMult <= 0){
      wristMult = .4;
    }

    //System.out.println"Wrist Multiplier" + wristMult);
  }
  
  public ArmAssembly(){
    
  }

  public void armInit(){
    backFingerOffset = backFinger_pot.get();
    frontFingerOffset = frontFinger_pot.get();
    leftIntakeOffset = leftIntake_pot.get();
    rightIntakeOffset = rightIntake_pot.get();
  }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new MoveArm());
  }

  public ArmPosition getArmPosition(){
    return armPosition;
  }

  public void setArmPosition(ArmPosition position){
    armPosition = position;
  }

  public void setArmEncoder(double angle){
    armFront_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    armFront_motor.setSelectedSensorPosition((int)(angle/(360f/4096f)));
  }

  public void setWristEncoder(double angle){
    wrist_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    wrist_motor.setSelectedSensorPosition((int)(angle/(360f/4096f)/(18f/40f)));
  }

  public void StartArmPID(double angle){
    armBack_motor.setSafetyEnabled(false);
    armFront_motor.setSafetyEnabled(false);
    armPID.setSetpoint(angle);
    //System.out.println"SETPOINT SET TO " + armPID.getSetpoint());
    armPID.enable();
  }

  public void StartJointPID(double angle, Joint joint){
    switch (joint){
      case ARM: {
        armBack_motor.setSafetyEnabled(false);
        armFront_motor.setSafetyEnabled(false);
        armPID.setSetpoint(angle);
        //System.out.println"SETPOINT SET TO " + armPID.getSetpoint());
        armPID.enable();
      }
      case WRIST: {
        wristPID.setSetpoint(angle);
        wristPID.enable();
      }
      case L_INTAKE: {
        leftIntakePID.setSetpoint(angle);
        leftIntakePID.enable();
      }
      case R_INTAKE: {
        rightIntakePID.setSetpoint(angle);
        rightIntakePID.enable();
      }
      case BACK_FINGER: {
        backFingerPID.setSetpoint(angle);
        backFingerPID.enable();
      }
      case FRONT_FINGER: {
        frontFingerPID.setSetpoint(angle);
        frontFingerPID.enable();
      }
    }
  }

  public void stopAll(){
    setWristMotor(0);
    setArmMotors(0);
    setLeftIntakeMotor(0);
    setRightIntakeMotor(0);
    setBackFingerMotor(0);
    setFrontFingerMotor(0);
  }

  public void StopArmPID(){
    //System.out.println"STOP ARM PID");
    armPID.disable();
    armPID.getPIDController().reset();
    Robot.pneumatics.brakeOn();
  }

  public void StopJointPID(Joint joint){
    switch (joint){
      case ARM:{
        //System.out.println"STOP ARM PID");
        armPID.disable();
        armPID.getPIDController().reset();
        Robot.pneumatics.brakeOn();
      }

      case WRIST: {
        wristPID.disable();
        wristPID.getPIDController().reset();
      }

      case L_INTAKE: leftIntakePID.disable();

      case R_INTAKE: rightIntakePID.disable();

      case BACK_FINGER: backFingerPID.disable();

      case FRONT_FINGER: frontFingerPID.disable();
    }
  }

  public void setJointMotor(VictorSPX victor, double output){
    victor.set(ControlMode.PercentOutput, output);
    if(victor == leftIntakeArm_motor && getJointAngle(Joint.L_INTAKE) < intakeThreshold){
      victor.set(ControlMode.PercentOutput, 0);
    } else if(victor == rightIntakeArm_motor && getJointAngle(Joint.R_INTAKE) < intakeThreshold){
      victor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void moveJoint(WPI_TalonSRX talon, double output){
    talon.set(ControlMode.PercentOutput, output);
  }

  public void moveJoint(Joint joint, double output){
    switch(joint){
      case WRIST: wrist_motor.set(ControlMode.PercentOutput, output);

      case ARM: {
        armBack_motor.set(ControlMode.PercentOutput, output);
        armFront_motor.set(ControlMode.PercentOutput, output);
      }

      case L_INTAKE: leftIntakeArm_motor.set(ControlMode.PercentOutput, output);

      case R_INTAKE: rightIntakeArm_motor.set(ControlMode.PercentOutput, output);

      case BACK_FINGER: backFinger_motor.set(ControlMode.PercentOutput, output);

      case FRONT_FINGER: frontFinger_motor.set(ControlMode.PercentOutput, output);
    }
  }

  public void setArmMotors(double output){
    //System.out.println"final output = " + output);
    armBack_motor.set(ControlMode.PercentOutput, output);
    armFront_motor.set(ControlMode.PercentOutput, -output);
  }

  public void setWristMotor(double output){
    System.out.println("Move wrist at " + output);
    wrist_motor.set(ControlMode.PercentOutput, output);
  }

  public void setBackFingerMotor(double output){
    backFinger_motor.configPeakOutputForward(.75);
    backFinger_motor.configPeakOutputReverse(-.75);
    backFinger_motor.set(ControlMode.PercentOutput, output);
  }

  public void setFrontFingerMotor(double output){
    frontFinger_motor.configPeakOutputForward(.75);
    frontFinger_motor.configPeakOutputReverse(-.75);
    frontFinger_motor.set(ControlMode.PercentOutput, output);
  }

  public void setRightIntakeMotor(double output){
    rightIntakeArm_motor.configPeakOutputForward(.75);
    rightIntakeArm_motor.configPeakOutputReverse(-.75);
    rightIntakeArm_motor.set(ControlMode.PercentOutput, output);
  }

  public void setLeftIntakeMotor(double output){
    leftIntakeArm_motor.configPeakOutputForward(.75);
    leftIntakeArm_motor.configPeakOutputReverse(-.75);
    leftIntakeArm_motor.set(ControlMode.PercentOutput, output);
  }

  public double getArmAngle(){
    double encRaw = 0.0;
    double angle = 0.0;

    armFront_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    encRaw = armFront_motor.getSelectedSensorPosition();
    angle = ((double)encRaw)*(360f/4096f);

    return angle;
  }

  public double getWristAngle(){
    double encRaw = 0;
    double angle = 0.0;

    wrist_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    encRaw = (double)wrist_motor.getSelectedSensorPosition();
    angle = (encRaw)*(18f/40f)*(360f/4096f);

    return angle;
  }

  public void resetJointEncoders(){
    wrist_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    wrist_motor.setSelectedSensorPosition(0);
    armFront_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    armFront_motor.setSelectedSensorPosition(0);

  }

  public double getBackFingerAngle(){
    return backFinger_pot.get() - backFingerOffset;
  }
  
  public double getJointAngle(Joint joint){
    switch (joint){
      case ARM: return getArmAngle();

      case WRIST: return getWristAngle();

      case L_INTAKE: return leftIntake_pot.get() - leftIntakeOffset;

      case R_INTAKE: return rightIntake_pot.get() - rightIntakeOffset;

      case BACK_FINGER: return backFinger_pot.get() - backFingerOffset;

      case FRONT_FINGER: return frontFinger_pot.get() - frontFingerOffset;
    }
    return -1;
  }

  public void cancelMotors(Joint joint){
    switch (joint){
      case ARM: armFront_motor.set(ControlMode.PercentOutput, 0);

      case WRIST: wrist_motor.set(ControlMode.PercentOutput, 0);

      case L_INTAKE: leftIntakeArm_motor.set(ControlMode.PercentOutput, 0);

      case R_INTAKE: rightIntakeArm_motor.set(ControlMode.PercentOutput, 0);

      case BACK_FINGER: backFinger_motor.set(ControlMode.PercentOutput, 0);

      case FRONT_FINGER: frontFinger_motor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void cancelAllMotors(){
    Robot.cleanStack();

    armFront_motor.set(ControlMode.PercentOutput, 0);

    wrist_motor.set(ControlMode.PercentOutput, 0);

    leftIntakeArm_motor.set(ControlMode.PercentOutput, 0);

    rightIntakeArm_motor.set(ControlMode.PercentOutput, 0);

    backFinger_motor.set(ControlMode.PercentOutput, 0);

    frontFinger_motor.set(ControlMode.PercentOutput, 0);

    armPID.disable();

    wristPID.disable();

    leftIntakePID.disable();

    rightIntakePID.disable();

    backFingerPID.disable();

    frontFingerPID.disable();
  }

  
}
