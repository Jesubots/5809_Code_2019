/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.PID;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.OI;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class PolarYPID extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  public PolarYPID() {
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
    int avg;
    avg = (Robot.driveTrain.getFLEncoder() + Robot.driveTrain.getBREncoder()) / 2;
    return avg; //returns average encoder value for front left and back right
  }

  @Override
  protected void usePIDOutput(double output) {
    Robot.driveTrain.frontLeft_motor.set(output * OI.getArmDir());
    Robot.driveTrain.backRight_motor.set(output * OI.getArmDir());
  }
}
