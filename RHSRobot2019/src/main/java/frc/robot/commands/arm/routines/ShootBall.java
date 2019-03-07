/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap.ArmPosition;
import frc.robot.RobotMap.BallTarget;
import frc.robot.commands.PID.DrivePolarEncoders;
import frc.robot.commands.PID.Lineup;
import frc.robot.commands.PID.PivotTurn;
import frc.robot.commands.arm.Fire;
import frc.robot.Robot;

public class ShootBall extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ShootBall() {

  }

  public ShootBall(BallTarget target){

    //AIM with LIMELIGHT
    addSequential(new PivotTurn(Robot.getVerticalOffset(), 2));
    addSequential(new Lineup(2));
    addSequential(new DrivePolarEncoders(0, 90, 2));
    //ARM POS, ADJUST DISTANCE, AND FIRE
    if(target == BallTarget.kMID){
      addParallel(new PositionArm(ArmPosition.kSHOOT_MID));
    } else if(target == BallTarget.kLOW){
      addParallel(new PositionArm(ArmPosition.kSHOOT_LOW));
    } else if(target == BallTarget.kCARGO){
      addParallel(new PositionArm(ArmPosition.kSHOOT_CARGO));
    }
    addSequential(new Fire(true));
  }
}
