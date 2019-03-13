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
import frc.robot.commands.PID.ManualArmPID;
import frc.robot.commands.PID.ManualBackFingerPID;
import frc.robot.commands.PID.ManualFrontFingerPID;
import frc.robot.commands.PID.ManualWristPID;

public class PositionArm extends CommandGroup {
  private double armAngle = Robot.armAssembly.getJointAngle(Joint.ARM);
  private double wristAngle = Robot.armAssembly.getJointAngle(Joint.WRIST);
  private double backFingerAngle = Robot.armAssembly.getJointAngle(Joint.BACK_FINGER);
  private double frontFingerAngle = Robot.armAssembly.getJointAngle(Joint.FRONT_FINGER);
  private double rightIntakeAngle = Robot.armAssembly.getJointAngle(Joint.R_INTAKE);
  private double leftIntakeAngle = Robot.armAssembly.getJointAngle(Joint.L_INTAKE);
  private int dir = 1;
  public PositionArm() {
    
  }

  public PositionArm(double arm, double wrist, double top, double bottom) {
    addParallel(new JointToAngle(Joint.ARM, arm, 2));
    addParallel(new JointToAngle(Joint.WRIST, wrist, 2));
    addParallel(new JointToAngle(Joint.BACK_FINGER, top, 2));
    addParallel(new JointToAngle(Joint.FRONT_FINGER, bottom, 2));
  }

  public PositionArm(ArmPosition position){
    dir = OI.getArmDir();
    //if we want to pickup/drop hatches
    if(position == ArmPosition.HATCH){
      armAngle = 90 - (45 * dir);
      wristAngle = armAngle;
      backFingerAngle = 90;
      frontFingerAngle = 90;
      leftIntakeAngle = 90;
      rightIntakeAngle = 90;
    } //if we want to pickup balls from the ground with the intake
    else if(position == ArmPosition.BALL_PICKUP){
      armAngle = 90 - (66 * dir);
      wristAngle = 180 + (86 * dir);
      backFingerAngle = 45;
      frontFingerAngle = 45;
      leftIntakeAngle = 90;
      rightIntakeAngle = 90;
    } //if we want to aim for the low rocket goal
    else if(position == ArmPosition.SHOOT_LOW){
      armAngle = 90 + (30 * dir);
      wristAngle = 180 + (15 * dir);
      backFingerAngle = 20;
      frontFingerAngle = 20;
      leftIntakeAngle = 90;
      rightIntakeAngle = 90;
    } //if we want to aim for the second level rocket goal
    else if(position == ArmPosition.SHOOT_MID){
      armAngle = 90 + (30 * dir);
      wristAngle = 180 + (15 * dir);
      backFingerAngle = 20;
      frontFingerAngle = 20;
      leftIntakeAngle = 90;
      rightIntakeAngle = 90;
    } //if we want to aim for the cargo ship goal
    else if(position == ArmPosition.SHOOT_CARGO){
      armAngle = 90 - (66 * dir);
      wristAngle = 180 + (86 * dir);
      backFingerAngle = 20;
      frontFingerAngle = 20;
      leftIntakeAngle = 90;
      rightIntakeAngle = 90;
    } //position we use at the start of the game
    else if(position == ArmPosition.DEFAULT){
      armAngle = 5;
      wristAngle = 0;
      backFingerAngle = 20;
      frontFingerAngle = 20;
      leftIntakeAngle = 90;
      rightIntakeAngle = 90;
    }
    else if(position == ArmPosition.HOLDING){
      armAngle = 90 - (10 * dir);
      wristAngle = 90 - (10 * dir);
      leftIntakeAngle = 90;
      rightIntakeAngle = 90;
    }
      addParallel(new ManualArmPID(armAngle, 2));
      addParallel(new ManualWristPID(wristAngle, 2));
      //addParallel(new ManualBackFingerPID(backFingerAngle, 2));
      //addParallel(new ManualFrontFingerPID(frontFingerAngle, 2));
      //addParallel(new JointToAngle(Joint.R_INTAKE, rightIntakeAngle, 2));
      //addParallel(new JointToAngle(Joint.L_INTAKE, leftIntakeAngle, 2));
      Robot.armAssembly.setArmPosition(position);
  }
}
