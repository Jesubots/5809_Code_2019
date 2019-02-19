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

public class PositionArm extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PositionArm() {
    
  }

  public PositionArm(ArmPosition position){
    //if we want to pickup/drop hatches
    if(position == ArmPosition.kHATCH){
      addParallel(new JointToAngle(Joint.kARM, 0, 2));
      addParallel(new JointToAngle(Joint.kWRIST, 0, 2));
      addParallel(new JointToAngle(Joint.kTOP_FINGER, 45, 2));
      addParallel(new JointToAngle(Joint.kBOTTOM_FINGER, -45, 2));
    } //if we want to pickup balls from the ground with the intake
    else if(position == ArmPosition.kBALL_PICKUP){
      addParallel(new JointToAngle(Joint.kARM, 0, 2));
      addParallel(new JointToAngle(Joint.kWRIST, -15, 2));
      addParallel(new JointToAngle(Joint.kTOP_FINGER, 0, 2));
      addParallel(new JointToAngle(Joint.kBOTTOM_FINGER, 0, 2));
    } //if we want to aim for the low cargo goal
    else if(position == ArmPosition.kSHOOT_LOW){
      addParallel(new JointToAngle(Joint.kARM, 15, 2));
      addParallel(new JointToAngle(Joint.kWRIST, 15, 2));
      addParallel(new JointToAngle(Joint.kTOP_FINGER, 0, 2));
      addParallel(new JointToAngle(Joint.kBOTTOM_FINGER, 0, 2));
    } //if we want to aim for the middle ship cargo goal
    else if(position == ArmPosition.kSHOOT_MID){
      addParallel(new JointToAngle(Joint.kARM, 20, 2));
      addParallel(new JointToAngle(Joint.kWRIST, 30, 2));
      addParallel(new JointToAngle(Joint.kTOP_FINGER, 0, 2));
      addParallel(new JointToAngle(Joint.kBOTTOM_FINGER, 0, 2));
    }
  }
}
