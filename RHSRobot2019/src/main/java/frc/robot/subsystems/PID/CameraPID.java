/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.PID;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class CameraPID extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  public CameraPID() {
    // Intert a subsystem name and PID values here
    super("CameraPID", RobotMap.CameraPIDMap.kP, RobotMap.CameraPIDMap.kI,
    RobotMap.CameraPIDMap.kD, RobotMap.CameraPIDMap.kF);
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
    return 0;//LIMELIGHT DISTANCE
  }

  @Override
  protected void usePIDOutput(double output) {
    Robot.driveTrain.backRight_motor.set(-output);
    Robot.driveTrain.frontRight_motor.set(output);
    Robot.driveTrain.frontLeft_motor.set(-output);
    Robot.driveTrain.backLeft_motor.set(output);
  }
}
