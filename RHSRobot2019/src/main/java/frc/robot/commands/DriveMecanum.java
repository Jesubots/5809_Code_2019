/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap.Joint;
import frc.robot.Robot;

public class DriveMecanum extends Command {
  private double xInput; //stick x axis
  private double yInput; //stick y axis
  private double tInput; //stick twist axis
  private double fr; //front-right motor value
  private double fl; //front-left motor value
  private double br; //back-right motor value
  private double bl; //back-left motor value
  private double theta;
  private double mag;
  Joystick stick = OI.driverStick; //joystick object
  Joystick stick2 = OI.buttonPanel;
  private float threshold = 0.2f;

  public DriveMecanum() {
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    xInput = stick.getX();
    yInput = stick.getY();
    tInput = stick.getZ();

    if(Math.abs(xInput) < .15){
      xInput = 0;
    }
    if(Math.abs(yInput) < .15){
      yInput = 0;
    }
    if(Math.abs(tInput) < .3){
      tInput = 0;
    }

    //System.out.print("arm angle = " +  Robot.armAssembly.getArmAngle());
    //System.out.println("wrist angle = " +  Robot.armAssembly.getWristAngle());
    
    Robot.driveTrain.mecanum.driveCartesian(-xInput * OI.getArmDir(), yInput * OI.getArmDir(), -tInput); //-Robot.driveTrain.ahrs.getYaw());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}