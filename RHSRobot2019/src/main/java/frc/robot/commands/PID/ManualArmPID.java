/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.PID;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap.Joint;

public class ManualArmPID extends Command {
  private double angle;
  private double output;
  private double target;
  private double error;
  private double timeout;
  
  public ManualArmPID() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  public ManualArmPID(double target, double timeout){
    this.target = target;
    this.timeout = timeout;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    error = 0.0;
    setTimeout(timeout);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    angle = Robot.armAssembly.getArmAngle();
    error = target - angle;
    output = error * .01;
    if(Math.abs(output) > 1f){
      output = 1f * Math.signum(output);
    }
    Robot.pneumatics.brakeOff();
    Robot.armAssembly.setArmMotors(output);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(error) < 5) || isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.armAssembly.setArmMotors(0);
    Robot.pneumatics.brakeOn();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.armAssembly.setArmMotors(0);
    Robot.pneumatics.brakeOn();
  }
}
