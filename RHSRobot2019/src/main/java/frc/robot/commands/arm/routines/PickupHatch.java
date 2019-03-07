/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap.ArmPosition;
import frc.robot.commands.PID.DrivePolarEncoders;
import frc.robot.commands.PID.Lineup;

public class PickupHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PickupHatch() {
    addSequential(new Lineup(2));
    addParallel(new PositionArm(ArmPosition.kHATCH));
    addSequential(new DrivePolarEncoders(0 /*ultrasonic value*/, 90, 2));
  }
}
