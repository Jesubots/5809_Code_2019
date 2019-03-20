/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.PID.DrivePolarEncoders;
import frc.robot.commands.arm.IntakeBall;
import frc.robot.commands.arm.RunIntakeEnds;

public class Climb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Climb() {
    addParallel(new RunIntakeEnds(5, -.4, -.4));
    addSequential(new Lift());
    addParallel(new DrivePolarEncoders(90, 100, 5));
    addParallel(new IntakeBall(1, 5));
  }
}
