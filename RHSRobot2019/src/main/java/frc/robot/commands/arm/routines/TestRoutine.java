/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap.ArmPosition;
import frc.robot.RobotMap.Joint;
import frc.robot.commands.PID.JointToAngle;
import frc.robot.commands.PID.ManualBackFingerPID;
import frc.robot.commands.PID.ManualFrontFingerPID;
import frc.robot.commands.PID.ManualLeftIntakePID;
import frc.robot.commands.PID.ManualRightIntakePID;
import frc.robot.commands.arm.RunIntakeEnds;

public class TestRoutine extends CommandGroup {
  /**
   * Add your docs here.
   */
  public TestRoutine() {
    addSequential(new PositionArm(ArmPosition.BALL_PICKUP));
    addSequential(new PositionArm(ArmPosition.HOLDING));
  }
}
