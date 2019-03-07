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
  Solenoid leftPunchForward_sol = new Solenoid(RobotMap.leftPunchForward_sol_port);
  Solenoid rightPunchForward_sol = new Solenoid(RobotMap.rightPunchForward_sol_port);
  Solenoid brakeForward_sol = new Solenoid(RobotMap.brakeForward_sol_port);
  Solenoid leftPunchReverse_sol = new Solenoid(RobotMap.leftPunchReverse_sol_port);
  Solenoid rightPunchReverse_sol = new Solenoid(RobotMap.rightPunchReverse_sol_port);
  Solenoid brakeReverse_sol = new Solenoid(RobotMap.brakeReverse_sol_port);

  public void punchOn(){
    leftPunchForward_sol.set(true);
    rightPunchForward_sol.set(true);
    leftPunchReverse_sol.set(false);
    rightPunchReverse_sol.set(false);
  }

  public void punchOff(){
    leftPunchForward_sol.set(false);
    rightPunchForward_sol.set(false);
    leftPunchReverse_sol.set(true);
    rightPunchReverse_sol.set(true);
    
  }

  public void brakeOn(){
    brakeForward_sol.set(true);
    brakeReverse_sol.set(false);
  }

  public void brakeOff(){
    brakeForward_sol.set(false);
    brakeReverse_sol.set(true);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
