/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.OI;
import frc.robot.Robot;

/***** how it works *****\
* take inputs: X, Y, and Twist
* The X input will become the X component of the vector.
* The Y input will become the Y component.
* The Twist input will turn the robot, as a tank drive turns.
* The Y component is as simple as passing that value to all motors...
* The X component is weird...
* The X input has to be passed normally to the top left and bottom right motors, 
* and opposite to the top right and bottom left motors.
* You add all these separate inputs, and that's really it.
* Vectors aren't *that* necessary, but they look real nice, so I'll use them.
\************************/

public class MecanumDrive extends Command {
  private double xInput; //stick x axis
  private double yInput; //stick y axis
  private double tInput; //stick twist axis
  private double fr; //front-right motor value
  private double fl; //front-left motor value
  private double br; //back-right motor value
  private double bl; //back-left motor value
  private Vector2d v = new Vector2d(0.0, 0.0); //2D vector that I'll make from the x/y inputs
  private double theta;
  private double mag;
  Joystick stick = OI.driverStick; //joystick object

  public MecanumDrive() {
    
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
    tInput = stick.getTwist();
    v = new Vector2d(xInput, yInput);
    theta = Math.atan(yInput/xInput);
    mag = Math.sqrt((xInput*xInput) + (yInput*yInput));
    fl = Math.cos(theta - 45);
    br = Math.cos(theta - 45);
    fr = Math.sin(theta - 45);
    bl = Math.sin(theta - 45);
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

    Robot.driveTrain.mecanumDrive(fl, fr, bl, br);
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