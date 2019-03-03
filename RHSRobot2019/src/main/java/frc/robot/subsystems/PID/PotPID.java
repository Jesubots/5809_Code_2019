/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.PID;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.Joint;

/**
 * Add your docs here.
 */
public class PotPID extends PIDSubsystem {
  private Joint joint;
  /**
   * Add your docs here.
   */
  public PotPID() {
    // Intert a subsystem name and PID values here
    super("PotPID", RobotMap.PotPIDMap.kP, RobotMap.PotPIDMap.kI, RobotMap.PotPIDMap.kD);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    if(joint == Joint.kARM){
      return Robot.armAssembly.getArmAngle();
    } else if(joint == Joint.kINTAKE){
      return ((Robot.armAssembly.leftIntake_pot.get() + Robot.armAssembly.rightIntake_pot.get()) / 2);
    } else if(joint == Joint.kTOP_FINGER){
      return Robot.armAssembly.topFinger_pot.get();
    } else if(joint == Joint.kBOTTOM_FINGER){
      return Robot.armAssembly.bottomFinger_pot.get();
    } else if(joint == Joint.kWRIST){
      return Robot.armAssembly.getWristAngle();
    } else {
      System.out.println("Cannot return PID input. No motor selected.");
      return -1;
    }
  }

  @Override
  protected void usePIDOutput(double output) {
    output *= .5;
    if(joint == Joint.kARM){
      Robot.armAssembly.moveJoint(Robot.armAssembly.armMaster_motor, output);
    } else if(joint == Joint.kINTAKE){
      Robot.armAssembly.moveJoint(Robot.armAssembly.rightIntakeArm_motor, output);
      Robot.armAssembly.moveJoint(Robot.armAssembly.leftIntakeArm_motor, output);
    } else if(joint == Joint.kTOP_FINGER){
      Robot.armAssembly.moveJoint(Robot.armAssembly.topFinger_motor, output);
    } else if(joint == Joint.kBOTTOM_FINGER){
      Robot.armAssembly.moveJoint(Robot.armAssembly.bottomFinger_motor, output);
    } else if(joint == Joint.kWRIST){
      Robot.armAssembly.moveJoint(Robot.armAssembly.wrist_motor, output);
    } else {
      System.out.println("Cannot use PID output. No motor selected.");
    }
  }

  public void setJoint(Joint inputJoint){
    joint = inputJoint;
  }
}
