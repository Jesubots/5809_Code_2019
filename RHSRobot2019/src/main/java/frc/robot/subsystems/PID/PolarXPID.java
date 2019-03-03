/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.PID;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.OI;
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
  }

  @Override
  public void initDefaultCommand() {
    
  }

  @Override
  protected double returnPIDInput() {
    int avg;
    avg = (Robot.driveTrain.getBLEncoder() + Robot.driveTrain.getFREncoder()) / 2;
    return avg; //returns average encoder value for front right and back left
  }

  @Override
  protected void usePIDOutput(double output) {
    Robot.driveTrain.frontRight_motor.set(output * OI.getArmDir());
    Robot.driveTrain.backLeft_motor.set(output * OI.getArmDir());
  }
}
