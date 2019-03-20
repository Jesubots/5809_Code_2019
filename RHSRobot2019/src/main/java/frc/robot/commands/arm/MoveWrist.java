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

public class MoveWrist extends Command {
  private double input = 0;
  public MoveWrist() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  public MoveWrist(double input){
    this.input = input;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Move wrist at " + input);
    setTimeout(.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println("Move wrist at " + input);
    Robot.armAssembly.setWristMotor(input);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.armAssembly.setWristMotor(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.armAssembly.setWristMotor(0);
  }
}
