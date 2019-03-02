/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap.ArmPosition;
import frc.robot.commands.arm.IntakeBall;

public class PickupBallGround extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PickupBallGround() {
    addSequential(new PositionArm(ArmPosition.kBALL_PICKUP));
    addSequential(new IntakeBall());
  }
}
