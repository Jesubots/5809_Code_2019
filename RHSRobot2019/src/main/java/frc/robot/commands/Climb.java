/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;
import frc.robot.RobotMap.Joint;
import frc.robot.commands.PID.DrivePolarEncoders;
import frc.robot.commands.PID.JointToAngle;
import frc.robot.commands.arm.IntakeBall;

public class Climb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Climb() {
    addSequential(new JointToAngle(Joint.kINTAKE, 95, 2));
    addParallel(new IntakeBall(1));
    addParallel(new DrivePolarEncoders(0, 90, 3));
    addSequential(new Lift());
    addParallel(new JointToAngle(Joint.kINTAKE, 0, RobotMap.climberTimeout));
    addParallel(new IntakeBall(1));
  }
}
