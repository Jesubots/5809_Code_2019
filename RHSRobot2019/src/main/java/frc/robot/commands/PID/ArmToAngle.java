/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.PID;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap.ArmPosition;
import frc.robot.RobotMap.Joint;

public class ArmToAngle extends Command {
  //joint is an enum value that tells the PID which motor and pot to use
  private Joint joint;
  private double angle;
  private double timeout;

  public ArmToAngle() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  public ArmToAngle(double angle, double timeout){
    this.angle = angle;
    this.timeout = timeout;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //we should be able to interrupt this
    setInterruptible(true);
    //make sure the command stops if it goes too long
		setTimeout(timeout);
    //initialize PID with angle and correct joint
    Robot.armAssembly.StartArmPID(angle);
    //set global position variable to NONE, will be changed to
    //correct one if this is called from PositionArm()
    Robot.armAssembly.setArmPosition(ArmPosition.NONE);
    Robot.armAssembly.armFront_motor.setNeutralMode(NeutralMode.Coast);
    Robot.armAssembly.armBack_motor.setNeutralMode(NeutralMode.Coast);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //finishes if the angle is within 5 degrees or the command times out
    return (Math.abs(angle - Robot.armAssembly.getArmAngle()) < 5f) || isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //stop the motor and PID when the command is done
    Robot.armAssembly.StopArmPID();
    Robot.armAssembly.armFront_motor.setNeutralMode(NeutralMode.Brake);
    Robot.armAssembly.armBack_motor.setNeutralMode(NeutralMode.Brake);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //stop the motor and PID when the command is interrupted
    Robot.armAssembly.StopArmPID();
    Robot.armAssembly.armFront_motor.setNeutralMode(NeutralMode.Brake);
    Robot.armAssembly.armBack_motor.setNeutralMode(NeutralMode.Brake);
  }
}
