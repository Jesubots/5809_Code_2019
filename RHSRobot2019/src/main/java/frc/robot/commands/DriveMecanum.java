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
    //the plan is to create a vector from x and y inputs
    //then to translate that vector into motor values
    //each motor has a certain vector associated with it
    //fr -- up-left
    //fl -- up-right
    //br -- up-right
    //bl -- up-left
    xInput = stick.getX();
    yInput = stick.getY();
    tInput = stick.getZ();
    /*
    if(Math.abs(stick.getX()) > threshold)
      xInput = stick.getX();
    else
      xInput = 0.0f;
    //System.out.println("x = " + xInput);
    if(Math.abs(stick.getY()) > threshold)
      yInput = stick.getY();
    else
      yInput = 0.0f;
    //System.out.println("y = " + yInput);
    if(Math.abs(stick.getZ()) > threshold)
      tInput = stick.getZ();
    else
      tInput = 0.0f;
    */
    //System.out.println("twist = " + yInput);
    theta = Math.atan(yInput/xInput);
    //System.out.println("angle = " + theta);
    mag = Math.sqrt((xInput*xInput) + (yInput*yInput));
    fl = Math.cos(theta - (Math.PI / 4));
    br = -Math.cos(theta - (Math.PI / 4));
    fr = -Math.sin(theta - (Math.PI / 4));
    bl = Math.sin(theta - (Math.PI / 4));

    if(Math.abs(stick.getZ()) > threshold){
      fl = -stick.getZ();
      br = -stick.getZ();
      fr = -stick.getZ();
      bl = -stick.getZ();
    }
    /*
    if(Math.abs(fl) == Math.sin(45)){
      fl = Math.signum(fl);
    }
    if(Math.abs(fr) == Math.sin(45)){
      fr = Math.signum(fr);
    }
    if(Math.abs(bl) == Math.sin(45)){
      bl = Math.signum(bl);
    }
    if(Math.abs(br) == Math.sin(45)){
      br = Math.signum(br);
    }
    */
    //System.out.println("final fl = " + fl);
    //System.out.println("final fr = " + fr);
    //System.out.println("final bl = " + bl);
    //%System.out.println("final br = " + br);
    //Robot.driveTrain.mecanumDrive(fl, fr, bl, br);

    //System.out.println("navx yaw = " + Robot.driveTrain.ahrs.getYaw());
    Robot.driveTrain.mecanum.driveCartesian(xInput, -yInput, -tInput); //-Robot.driveTrain.ahrs.getYaw());
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