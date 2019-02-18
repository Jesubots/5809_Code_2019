/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class PolarXPID extends PIDSubsystem {
  /**
   * 
   */
  boolean direction;
  public PolarXPID() {
    // Intert a subsystem name and PID values here
    super("PolarPID", RobotMap.PolarPIDMap.kP, RobotMap.PolarPIDMap.kI,
				RobotMap.PolarPIDMap.kD, RobotMap.PolarPIDMap.kF);
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
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return 0.0; //should return encoder values for front right and back left
  }

  @Override
  protected void usePIDOutput(double output) {
    Robot.driveTrain.setFrontRight(output);
    Robot.driveTrain.setBackLeft(output);
  }
}
