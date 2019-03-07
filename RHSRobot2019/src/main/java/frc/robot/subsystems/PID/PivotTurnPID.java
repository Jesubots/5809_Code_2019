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
public class PivotTurnPID extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  public PivotTurnPID() {
    // Intert a subsystem name and PID values here
    super("PivotTurnPID", RobotMap.PivotTurnPIDMap.kP, RobotMap.PivotTurnPIDMap.kI,
    RobotMap.PivotTurnPIDMap.kD, RobotMap.PivotTurnPIDMap.kF);
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
    return Robot.driveTrain.ahrs.getYaw();
  }

  @Override
  protected void usePIDOutput(double output) {
    Robot.driveTrain.backRight_motor.set(-output);
    Robot.driveTrain.frontRight_motor.set(-output);
    Robot.driveTrain.frontLeft_motor.set(output);
    Robot.driveTrain.backLeft_motor.set(output);
  }
}
