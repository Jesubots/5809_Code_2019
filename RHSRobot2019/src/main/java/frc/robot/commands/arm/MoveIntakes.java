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

public class MoveIntakes extends Command {
  private double input;
  public MoveIntakes() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  public MoveIntakes(double input){
    this.input = input;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(1);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.armAssembly.setLeftIntakeMotor(-input);
    Robot.armAssembly.setRightIntakeMotor(input);
    System.out.println("left arm voltage = " + Robot.armAssembly.leftIntakeArm_motor.getBusVoltage());
    System.out.println("right arm voltage = " + Robot.armAssembly.rightIntakeArm_motor.getBusVoltage());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.armAssembly.setLeftIntakeMotor(0);
    Robot.armAssembly.setRightIntakeMotor(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.armAssembly.setLeftIntakeMotor(0);
    Robot.armAssembly.setRightIntakeMotor(0);
  }
}
