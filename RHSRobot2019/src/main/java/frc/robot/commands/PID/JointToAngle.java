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

public class JointToAngle extends Command {
  //joint is an enum value that tells the PID which motor and pot to use
  private Joint joint;
  private double angle;
  private double timeout;

  public JointToAngle() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  public JointToAngle(Joint joint, double angle, double timeout){
    this.angle = angle;
    this.joint = joint;
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
    Robot.armAssembly.JointToAnglePID(angle, joint);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //finishes if the angle is within 5 degrees or the command times out
    return (Math.abs(angle - Robot.armAssembly.getPotAngle(joint)) < 5) || isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //stop the motor when the command is done
    cancelMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //stop the motor when the command is interrupted
    cancelMotors();
  }

  private void cancelMotors(){
    //all this does is turn off whichever motor is being used by the PID
    if(joint == Joint.kARM){
      Robot.armAssembly.moveJoint(Robot.armAssembly.armMaster_motor, 0);
    } else if(joint == Joint.kINTAKE){
      Robot.armAssembly.moveJoint(Robot.armAssembly.rightIntakeArm_motor, 0);
    } else if(joint == Joint.kTOP_FINGER){
      Robot.armAssembly.moveJoint(Robot.armAssembly.topFinger_motor, 0);
    } else if(joint == Joint.kBOTTOM_FINGER){
      Robot.armAssembly.moveJoint(Robot.armAssembly.bottomFinger_motor, 0);
    } else if(joint == Joint.kWRIST){
      Robot.armAssembly.moveJoint(Robot.armAssembly.wrist_motor, 0);
    } else {
      System.out.println("Cannot cancel motors. No motor selected.");
    }
  }
}
