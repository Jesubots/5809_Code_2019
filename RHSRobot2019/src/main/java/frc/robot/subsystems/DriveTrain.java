/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.DriveMecanum;
import frc.robot.subsystems.PID.CameraPID;
import frc.robot.subsystems.PID.PivotTurnPID;
import frc.robot.subsystems.PID.PolarXPID;
import frc.robot.subsystems.PID.PolarYPID;

public class DriveTrain extends Subsystem {
  public WPI_TalonSRX frontRight_motor = new WPI_TalonSRX(RobotMap.frontRight_port);
  public WPI_TalonSRX frontLeft_motor = new WPI_TalonSRX(RobotMap.frontLeft_port);
  public WPI_TalonSRX backLeft_motor = new WPI_TalonSRX(RobotMap.backLeft_port);
  public WPI_TalonSRX backRight_motor = new WPI_TalonSRX(RobotMap.backRight_port);
  public AHRS ahrs = new AHRS(Port.kMXP);
  public Ultrasonic us = new Ultrasonic(RobotMap.ultrasonicOut_port, RobotMap.ultrasonicIn_port, Unit.kInches);
  Joystick stick = OI.driverStick;

  public MecanumDrive mecanum = new MecanumDrive(frontLeft_motor, backLeft_motor, frontRight_motor, backRight_motor);

  public PolarXPID encXPID = new PolarXPID();
  public PolarYPID encYPID = new PolarYPID();
  public PivotTurnPID pivotTurnPID = new PivotTurnPID();
  public CameraPID cameraPID = new CameraPID();

  public DriveTrain() {
    frontRight_motor.setNeutralMode(NeutralMode.Brake);
    frontLeft_motor.setNeutralMode(NeutralMode.Brake);
    backRight_motor.setNeutralMode(NeutralMode.Brake);
    backLeft_motor.setNeutralMode(NeutralMode.Brake);
    frontLeft_motor.setSafetyEnabled(true);
    frontRight_motor.setSafetyEnabled(true);
    backLeft_motor.setSafetyEnabled(true);
    backRight_motor.setSafetyEnabled(true);
    

    ahrs.zeroYaw();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveMecanum());
  }

  public void mecanumDrive(double fr, double fl, double br, double bl) {
    // frontRight.set(ControlMode.PercentOutput, fr);
    // frontLeft.set(ControlMode.PercentOutput, fl);
    // backRight.set(ControlMode.PercentOutput, br);
    // backLeft.set(ControlMode.PercentOutput, bl);
  }

  public void setBrakeMode(){
    frontRight_motor.setNeutralMode(NeutralMode.Brake);
    frontLeft_motor.setNeutralMode(NeutralMode.Brake);
    backRight_motor.setNeutralMode(NeutralMode.Brake);
    backLeft_motor.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode(){
    frontRight_motor.setNeutralMode(NeutralMode.Coast);
    frontLeft_motor.setNeutralMode(NeutralMode.Coast);
    backRight_motor.setNeutralMode(NeutralMode.Coast);
    backLeft_motor.setNeutralMode(NeutralMode.Coast);
  }

  public void DrivePolarXPID(double distance) {
    System.out.println("EncoderPID (x) started...");
    encXPID.setSetpoint(distance);
    encXPID.enable();
  }

  public void DrivePolarYPID(double distance) {
    System.out.println("EncoderPID (y) started...");
    encYPID.setSetpoint(distance);
    encYPID.enable();
  }

  public void StartPivotTurnPID(double angle) {
    System.out.println("PivotTurnPID started...");
    pivotTurnPID.setSetpoint(angle);
    pivotTurnPID.enable();
  }

  public void StartCameraPID() {
    System.out.println("CameraPID started...");
    cameraPID.setSetpoint(0);//limelight angle);
    cameraPID.enable();
  }

  public void StopCameraPID() {
    cameraPID.disable();
  }

  public void StopPivotTurnPID() {
    pivotTurnPID.disable();
  }

  public void StopPolarXPID() {
    encXPID.disable();
  }

  public void StopPolarYPID() {
    encYPID.disable();
  }

  //encoder position methods
  public int getFREncoder(){
    frontRight_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    return frontRight_motor.getSelectedSensorPosition();
  }

  public int getFLEncoder(){
    frontLeft_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    return frontLeft_motor.getSelectedSensorPosition();
  }

  public int getBREncoder(){
    backRight_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    return backRight_motor.getSelectedSensorPosition();
  }

  public int getBLEncoder(){
    backLeft_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    return backLeft_motor.getSelectedSensorPosition();
  }

  //reset all encoder values to zero
  public void resetEncoders(){
    frontRight_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    frontLeft_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    backRight_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    backLeft_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    frontRight_motor.setSelectedSensorPosition(0);
    frontLeft_motor.setSelectedSensorPosition(0);
    backLeft_motor.setSelectedSensorPosition(0);
    backLeft_motor.setSelectedSensorPosition(0);
  }
}
