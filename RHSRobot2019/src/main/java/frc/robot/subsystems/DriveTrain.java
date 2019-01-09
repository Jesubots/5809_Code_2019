/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;


public class DriveTrain extends Subsystem {
  //thinking mecanum drive...
  //Mecanum drive programming is, in essence, vector addition based on the
  //inputs from the joystick and the way those inputs translate into a 
  //vector usable in driving the robot

  public DriveTrain(){

  }

  @Override
  public void initDefaultCommand() {
    //default command
  }

  public void mecanumDrive(double frontRight, double frontLeft, double backRight, double backLeft){

  }
}
