/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.OI;
import frc.robot.commands.DriveMecanum;

public class DriveTrain extends Subsystem {
  // thinking mecanum drive...
  // Mecanum drive programming is, in essence, vector addition based on the
  // inputs from the joystick and the way those inputs translate into a
  // vector usable in driving the robot
  public WPI_TalonSRX frontRight = new WPI_TalonSRX(0);
  public WPI_TalonSRX frontLeft = new WPI_TalonSRX(3);
  public WPI_TalonSRX backLeft = new WPI_TalonSRX(1);
  public WPI_TalonSRX backRight = new WPI_TalonSRX(2);
  public AHRS ahrs = new AHRS(Port.kMXP);
  private double[] motorValues = new double[4];
  Joystick stick = OI.driverStick;

  public MecanumDrive mecanum = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

  public PolarXPID encXPID;
  public PolarYPID encYPID;

  public DriveTrain() {
    // frontRight.setNeutralMode(NeutralMode.Brake);
    // frontLeft.setNeutralMode(NeutralMode.Brake);
    // backRight.setNeutralMode(NeutralMode.Brake);
    // backLeft.setNeutralMode(NeutralMode.Brake);
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

  public double[] getMotorValues(){
    return motorValues;
  }

  public void setMotorValues(double[] newMotorValues){
    motorValues = newMotorValues;
  }

  public void setFrontLeft(double input){
    motorValues[0] = input;
  }

  public void setFrontRight(double input){
    motorValues[1] = input;
  }

  public void setBackLeft(double input){
    motorValues[2] = input;
  }

  public void setBackRight(double input){
    motorValues[3] = input;
  }

}
