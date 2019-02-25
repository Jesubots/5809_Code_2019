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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.OI;
import frc.robot.commands.DriveMecanum;
import frc.robot.subsystems.PID.PolarXPID;
import frc.robot.subsystems.PID.PolarYPID;

public class DriveTrain extends Subsystem {
  public WPI_TalonSRX frontRight = new WPI_TalonSRX(0);
  public WPI_TalonSRX frontLeft = new WPI_TalonSRX(3);
  public WPI_TalonSRX backLeft = new WPI_TalonSRX(1);
  public WPI_TalonSRX backRight = new WPI_TalonSRX(2);
  public AHRS ahrs = new AHRS(Port.kMXP);
  Joystick stick = OI.driverStick;

  public MecanumDrive mecanum = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

  public PolarXPID encXPID = new PolarXPID();
  public PolarYPID encYPID = new PolarYPID();

  public DriveTrain() {
    frontRight.setNeutralMode(NeutralMode.Brake);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    frontLeft.setSafetyEnabled(false);
    frontRight.setSafetyEnabled(false);
    backLeft.setSafetyEnabled(false);
    backRight.setSafetyEnabled(false);

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
    frontRight.setNeutralMode(NeutralMode.Brake);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode(){
    frontRight.setNeutralMode(NeutralMode.Coast);
    frontLeft.setNeutralMode(NeutralMode.Coast);
    backRight.setNeutralMode(NeutralMode.Coast);
    backLeft.setNeutralMode(NeutralMode.Coast);
  }

  public void DrivePolarXPID(double distance) {
    System.out.println("Encoder PID (x) started...");
    encXPID.setSetpoint(distance);
    encXPID.enable();
  }

  public void DrivePolarYPID(double distance) {
    System.out.println("Encoder PID (y) started...");
    encYPID.setSetpoint(distance);
    encYPID.enable();
  }

  public void StopPolarXPID() {
    encXPID.disable();
  }

  public void StopPolarYPID() {
    encYPID.disable();
  }

  //encoder position methods
  public int getFREncoder(){
    frontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    return frontRight.getSelectedSensorPosition();
  }

  public int getFLEncoder(){
    frontLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    return frontLeft.getSelectedSensorPosition();
  }

  public int getBREncoder(){
    backRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    return backRight.getSelectedSensorPosition();
  }

  public int getBLEncoder(){
    backLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    return backLeft.getSelectedSensorPosition();
  }

  //reset all encoder values to zero
  public void resetEncoders(){
    frontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    frontLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    backRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    backLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    frontRight.setSelectedSensorPosition(0);
    frontLeft.setSelectedSensorPosition(0);
    backLeft.setSelectedSensorPosition(0);
    backLeft.setSelectedSensorPosition(0);
  }
}
