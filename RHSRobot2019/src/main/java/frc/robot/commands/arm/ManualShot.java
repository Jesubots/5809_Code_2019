/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap.Joint;
import frc.robot.commands.PID.JointToAngle;

public class ManualShot extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ManualShot() {
    addSequential(new JointToAngle(Joint.kBOTTOM_FINGER, 145, .5));
    addParallel(new JointToAngle(Joint.kTOP_FINGER, 145, .5));
    addSequential(new Fire(true));
  }
}
