/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RunIntakeEnds extends Command {
  private double timeout;
  private double rightOutput;
  private double leftOutput;

  public RunIntakeEnds() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  public RunIntakeEnds(double timeout, double left, double right){
    this.timeout = timeout;
    this.leftOutput = left;
    this.rightOutput = right;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(timeout);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.armAssembly.setJointMotor(Robot.armAssembly.leftIntakeEnd_motor, leftOutput);
    Robot.armAssembly.setJointMotor(Robot.armAssembly.rightIntakeEnd_motor, rightOutput);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.armAssembly.setJointMotor(Robot.armAssembly.leftIntakeEnd_motor, 0);
    Robot.armAssembly.setJointMotor(Robot.armAssembly.rightIntakeEnd_motor, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.armAssembly.setJointMotor(Robot.armAssembly.leftIntakeEnd_motor, 0);
    Robot.armAssembly.setJointMotor(Robot.armAssembly.rightIntakeEnd_motor, 0);
  }
}
