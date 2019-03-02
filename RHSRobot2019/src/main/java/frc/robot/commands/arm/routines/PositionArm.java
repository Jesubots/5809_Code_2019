/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.OI;
import frc.robot.RobotMap.ArmPosition;
import frc.robot.RobotMap.Joint;
import frc.robot.commands.PID.JointToAngle;

public class PositionArm extends CommandGroup {
  private double armAngle = 0;
  private double wristAngle = 0;
  private double topFingerAngle = 0;
  private double bottomFingerAngle = 0;
  private int dir = 1;
  public PositionArm() {
    
  }

  public PositionArm(double arm, double wrist, double top, double bottom) {
    addParallel(new JointToAngle(Joint.kARM, arm, 2));
    addParallel(new JointToAngle(Joint.kWRIST, wrist, 2));
    addParallel(new JointToAngle(Joint.kTOP_FINGER, top, 2));
    addParallel(new JointToAngle(Joint.kBOTTOM_FINGER, bottom, 2));
  }

  public PositionArm(ArmPosition position){
    dir = OI.getArmDir();
    //if we want to pickup/drop hatches
    if(position == ArmPosition.kHATCH){
      armAngle = 32 * OI.getArmDir();
      wristAngle = armAngle + 90;
      topFingerAngle = 45;
      bottomFingerAngle = 135;
    } //if we want to pickup balls from the ground with the intake
    else if(position == ArmPosition.kBALL_PICKUP){
      armAngle = 90 + (66 * dir);
      wristAngle = 180 + (86 * dir);
      topFingerAngle = 45;
      bottomFingerAngle = 135;
    } //if we want to aim for the low cargo goal
    else if(position == ArmPosition.kSHOOT_LOW){
      armAngle = 0;
      wristAngle = 0;
      topFingerAngle = 0;
      bottomFingerAngle = 0;
    } //if we want to aim for the middle ship cargo goal
    else if(position == ArmPosition.kSHOOT_MID){
      armAngle = 0;
      wristAngle = 0;
      topFingerAngle = 0;
      bottomFingerAngle = 0;
    }
      addParallel(new JointToAngle(Joint.kARM, 20, 2));
      addParallel(new JointToAngle(Joint.kWRIST, 30, 2));
      addParallel(new JointToAngle(Joint.kTOP_FINGER, 0, 2));
      addParallel(new JointToAngle(Joint.kBOTTOM_FINGER, 0, 2));
  }
}
