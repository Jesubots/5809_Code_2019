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

public class MoveFingers extends Command {
  private double input;
  public MoveFingers() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  public MoveFingers(double input){
    this.input = input;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.armAssembly.frontFinger_motor.set(ControlMode.PercentOutput, input * .25);
    Robot.armAssembly.backFinger_motor.set(ControlMode.PercentOutput, input * .25);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.armAssembly.frontFinger_motor.set(ControlMode.PercentOutput, 0);
    Robot.armAssembly.backFinger_motor.set(ControlMode.PercentOutput, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.armAssembly.frontFinger_motor.set(ControlMode.PercentOutput, 0);
    Robot.armAssembly.backFinger_motor.set(ControlMode.PercentOutput, 0);
  }
}
