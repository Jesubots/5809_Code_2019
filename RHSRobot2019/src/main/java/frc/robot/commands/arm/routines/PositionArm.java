/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap.ArmPosition;
import frc.robot.RobotMap.Joint;
import frc.robot.commands.PID.JointToAngle;
import frc.robot.commands.arm.Brake;

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
      armAngle = 90 + (58 * dir);
      wristAngle = armAngle + 90;
      topFingerAngle = 45;
      bottomFingerAngle = 135;
    } //if we want to pickup balls from the ground with the intake
    else if(position == ArmPosition.kBALL_PICKUP){
      armAngle = 90 + (66 * dir);
      wristAngle = 180 + (86 * dir);
      topFingerAngle = 45;
      bottomFingerAngle = 135;
    } //if we want to aim for the low rocket goal
    else if(position == ArmPosition.kSHOOT_LOW){
      armAngle = 90 - (30 * dir);
      wristAngle = 180 + (15 * dir);
      topFingerAngle = 45;
      bottomFingerAngle = 135;
    } //if we want to aim for the second level rocket goal
    else if(position == ArmPosition.kSHOOT_MID){
      armAngle = 90 - (30 * dir);
      wristAngle = 180 + (15 * dir);
      topFingerAngle = 45;
      bottomFingerAngle = 135;
    } //if we want to aim for the cargo ship goal
    else if(position == ArmPosition.kSHOOT_CARGO){
      armAngle = 90 + (66 * dir);
      wristAngle = 180 + (86 * dir);
      topFingerAngle = 45;
      bottomFingerAngle = 135;
    } //position we use at the start of the game
    else if(position == ArmPosition.kDEFAULT){
      armAngle = 5;
      wristAngle = 0;
      topFingerAngle = 45;
      bottomFingerAngle = 135;
    }
    else if(position == ArmPosition.kHOLDING){
      armAngle = 90 + (10 * dir);
      wristAngle = 90 - (10 * dir);
    }
      Robot.armAssembly.setArmPosition(position);
      addParallel(new JointToAngle(Joint.kARM, armAngle, 2));
      addParallel(new JointToAngle(Joint.kWRIST, wristAngle, 2));
      addParallel(new JointToAngle(Joint.kTOP_FINGER, topFingerAngle, 2));
      addParallel(new JointToAngle(Joint.kBOTTOM_FINGER, bottomFingerAngle, 2));
      addSequential(new Brake());
  }
}
