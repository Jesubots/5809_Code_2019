/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap.ArmPosition;
import frc.robot.commands.PID.DrivePolarEncoders;
import frc.robot.commands.PID.Lineup;
import frc.robot.commands.PID.PivotTurn;
import frc.robot.commands.arm.Fire;

public class PlaceHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  private double angle = Robot.getVerticalOffset();
  private double distance = 0; //ASSIGN ULTRASONIC VALUE
  public PlaceHatch() {
    addSequential(new PivotTurn(angle, 3));
    addSequential(new Lineup(2));
    addParallel(new PositionArm(ArmPosition.kHATCH));
    addSequential(new DrivePolarEncoders(distance, 90, 2));
    addSequential(new Fire(false));
  }
}
