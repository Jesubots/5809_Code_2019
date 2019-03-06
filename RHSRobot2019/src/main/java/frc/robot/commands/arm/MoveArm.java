/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.Joint;

public class MoveArm extends Command {
  private double input;

  public MoveArm() {
    requires(Robot.armAssembly);
  }

  public MoveArm(double input){
    this.input = input;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(1);
    Robot.armAssembly.StartPotPID(90 + Robot.armAssembly.getArmAngle(), Joint.kWRIST);
    System.out.println("Move Arm");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.armAssembly.armMaster_motor.set(ControlMode.Current, input * OI.getArmDir());
    //continuously sets the PID target to the correct angle
    Robot.armAssembly.wristPID.setSetpoint(90 + Robot.armAssembly.getArmAngle());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.armAssembly.StopPotPID(Joint.kWRIST);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.armAssembly.StopPotPID(Joint.kWRIST);
  }
}
