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
public class ArmPID extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  public ArmPID() {
    super("ArmPID", RobotMap.ArmPIDMap.kP, RobotMap.ArmPIDMap.kI, 
    RobotMap.ArmPIDMap.kD);
  }

  @Override
  public void initDefaultCommand() {
  }

  @Override
  protected double returnPIDInput() {
    return Robot.armAssembly.getJointAngle(Joint.ARM);
  }

  @Override
  protected void usePIDOutput(double output) {
    //System.out.println"run arm at " + output);
    //System.out.println"angle = " + Robot.armAssembly.getJointAngle(Joint.ARM));
    Robot.pneumatics.brakeOff();
    Robot.armAssembly.setArmMotors(output);
  }
}
