/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Fire extends Command {
  private boolean projectile = true;
  private boolean finished = false;
  protected boolean mag = true;

  public Fire() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  /*
  * takes a boolean - true is for firing ball, false is for hatch
  */
  public Fire(boolean input){
    projectile = input;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.armAssembly.setBackFingerMotor(-.25);
    Robot.armAssembly.setFrontFingerMotor(-.25);
    Timer.delay(.05);
    Robot.armAssembly.setBackFingerMotor(0);
    Robot.armAssembly.setFrontFingerMotor(0);
    Robot.pneumatics.punchOn();
    Timer.delay(1);
    Robot.pneumatics.punchOff();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.pneumatics.punchOff();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    
  }
}
