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
  private boolean compensating = false;

  public MoveArm() {
    requires(Robot.armAssembly);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    input = OI.buttonPanel.getY() * .25;
    if(Math.abs(input) < .05){
      Robot.armAssembly.StopPotPID(Joint.kWRIST);
      //turn this boolean off to let the PID turn on when an input is great enough
      compensating = false;
    } else {
      Robot.pneumatics.brakeOff();
      if(!compensating){
        //input turns on only if it isn't already on (hence the boolean)
        //PID is turned on for the wrist to go to the angle perpendicular to the ground
        Robot.armAssembly.StartPotPID(90 + Robot.armAssembly.getArmAngle(), Joint.kWRIST);
        compensating = true;
      }
      Robot.armAssembly.armMaster_motor.set(ControlMode.Current, input);
      //continuously sets the PID target to the correct angle
      Robot.armAssembly.potPID.setSetpoint(90 + Robot.armAssembly.getArmAngle());
    }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
