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

public class PivotTurn extends Command {
  
  private double angle;
  private double driveTimeout;
  private double xDist;
  private double yDist;

  public PivotTurn() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }
  
  /**
  * Input polar coordinates, i.e. distance and angle (in radians), and the robot will use the Encoders and a PID to
  * move the correct distance in the correct direction
  */
  public PivotTurn(double angle, double driveTimeout){
    //converts inches to encoder counts
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
    Robot.driveTrain.StartPivotTurnPID(angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(angle - Robot.driveTrain.ahrs.getYaw()) < 5 || isTimedOut());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.StopPivotTurnPID();
    Robot.driveTrain.StopPivotTurnPID();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.StopPivotTurnPID();
    Robot.driveTrain.StopPivotTurnPID();
  }
}
