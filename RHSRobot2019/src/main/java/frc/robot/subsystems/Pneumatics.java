/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Pneumatics extends Subsystem {
  Solenoid leftPunch_sol = new Solenoid(RobotMap.leftPunch_sol_port);
  Solenoid rightPunch_sol = new Solenoid(RobotMap.rightPunch_sol_port);
  Solenoid brake_sol = new Solenoid(RobotMap.brake_sol_port);

  public void punch(){
    leftPunch_sol.set(true);
    rightPunch_sol.set(true);
  }

  public void brakeOn(){
    brake_sol.set(true);
  }

  public void brakeOff(){
    brake_sol.set(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
