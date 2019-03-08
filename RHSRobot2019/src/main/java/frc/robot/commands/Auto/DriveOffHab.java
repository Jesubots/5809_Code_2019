/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap.ArmPosition;
import frc.robot.commands.DriveMecanum;
import frc.robot.commands.PID.DrivePolarEncoders;
import frc.robot.commands.arm.routines.PositionArm;

public class DriveOffHab extends CommandGroup {
  /**
   * Add your docs here.
   */
  public DriveOffHab() {
    addSequential(new PositionArm(ArmPosition.kDEFAULT));
    addSequential(new DrivePolarEncoders(90, 36, 3));
    addSequential(new DriveMecanum());
  }
}
