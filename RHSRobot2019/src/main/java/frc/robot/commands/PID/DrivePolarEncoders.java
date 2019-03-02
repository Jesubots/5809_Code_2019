/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.PID;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DrivePolarEncoders extends Command {
  
  private double distance;
  private double angle;
  private double driveTimeout;
  private double xDist;
  private double yDist;

  public DrivePolarEncoders() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }
  
  /**
  * Input polar coordinates, i.e. distance and angle (in radians), and the robot will use the Encoders and a PID to
  * move the correct distance in the correct direction
  */
  public DrivePolarEncoders(double distance, double angle, double driveTimeout){
    //converts inches to encoder counts
    this.distance = distance * RobotMap.ENCODER_CONVERSION_CONSTANT; 
    this.angle = angle;
    this.driveTimeout = driveTimeout;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //we should be able to interrupt this
    setInterruptible(true);
    //make sure the command ends if it's not done fast enough
		setTimeout(driveTimeout);
    
    //convert polar coordinates to X and Y components
    xDist = distance * Math.cos(angle - (Math.PI / 4));
    yDist = distance * Math.sin(angle - (Math.PI / 4));
    //input those components into the PID init in DriveTrain
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
    //calculate distances traveled on X and Y axes by averaging encoder values
    double xAvg = (double)(Robot.driveTrain.getFREncoder() + Robot.driveTrain.getBLEncoder()) / 2.0;
    double yAvg = (double)(Robot.driveTrain.getFLEncoder() + Robot.driveTrain.getBREncoder()) / 2.0;
    
    //some checks to see if the robot is within 100 encoder counts of the target
    boolean finishedX = (Math.abs(xDist - xAvg) < 100);
    boolean finishedY = (Math.abs(yDist - yAvg) < 100);

    //finished if X and Y are within 100 counts or if the command times out
    return ((finishedX && finishedY) || isTimedOut());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.StopPolarXPID();
    Robot.driveTrain.StopPolarYPID();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.StopPolarXPID();
    Robot.driveTrain.StopPolarYPID();
  }
}
