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
public class RightIntakePID extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  public RightIntakePID() {
    super("RightIntakePID", RobotMap.IntakePIDMap.kP, RobotMap.IntakePIDMap.kI, 
    RobotMap.IntakePIDMap.kD);
  }

  @Override
  public void initDefaultCommand() {
  }

  @Override
  protected double returnPIDInput() {
    return Robot.armAssembly.getJointAngle(Joint.R_INTAKE);
  }

  @Override
  protected void usePIDOutput(double output) {
    Robot.armAssembly.setRightIntakeMotor(output);
  }
}
