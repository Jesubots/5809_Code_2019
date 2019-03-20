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
import frc.robot.subsystems.ArmAssembly;

public class ManualArmPID extends Command {
  private double angle;
  private double output;
  private double target;
  private double error;
  private double timeout;
  private double kP;
  private ArmAssembly aa = Robot.armAssembly;
  
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
    if(angle < 0){
      if(angle < target && Math.abs(angle) > 30){
        System.out.println("KP BIG");
        kP = .05;
      } else {
        System.out.println("KP SMOL");
        kP = .004;
      }
    } else {
      if(angle > target && Math.abs(angle) > 30){
        System.out.println("KP BIG");
        kP = .05;
      } else {
        System.out.println("KP SMOL");
        kP = .004;
      }
    }
    output = error * kP;
    if(Math.abs(output) > .8f){
      output = .8f * Math.signum(output);
    }
    System.out.println("arm angle = " + angle);
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
    System.out.println("Finished Arm PID");
    Robot.armAssembly.setArmMotors(0);
    Robot.pneumatics.brakeOn();
    Robot.pneumatics.disableBrake();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.armAssembly.setArmMotors(0);
    Robot.pneumatics.brakeOn();
    Robot.pneumatics.disableBrake();
  }
}
