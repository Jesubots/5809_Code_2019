/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DrivePolarEncoders extends Command {
  
  private double distance;
  private double angle;
  private double driveTimeout;
  public DrivePolarEncoders() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }
  
  /*
  * Input polar coordinates, i.e. distance and angle (in radians), and the robot will use the Encoders and a PID to
  * move the correct distance in the correct direction
  */
  public DrivePolarEncoders(double distance, double angle, double driveTimeout){
    this.distance = distance;
    this.angle = angle;
    this.driveTimeout = driveTimeout;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setInterruptible(true);
		setTimeout(driveTimeout);

    double xDist = distance * Math.cos(angle - (Math.PI / 4));
    double yDist = distance * Math.cos(angle - (Math.PI / 4));
    Robot.driveTrain.DrivePolarXPID(xDist);
    Robot.driveTrain.DrivePolarXPID(yDist);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(distance/* - get encoder distance */) < 100) || isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
