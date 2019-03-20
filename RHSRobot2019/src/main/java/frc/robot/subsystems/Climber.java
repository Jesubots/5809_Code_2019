/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  public VictorSPX rightClimber_motor = new VictorSPX(RobotMap.rightClimber_motor_port);
  public VictorSPX leftClimber_motor = new VictorSPX(RobotMap.leftClimber_motor_port);
  public DigitalInput leftClimberLimit = new DigitalInput(RobotMap.climberLimitSwitch_port);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void lift(){
    //if(!climberLimit.get()){
      rightClimber_motor.set(ControlMode.PercentOutput, -1);
      leftClimber_motor.set(ControlMode.PercentOutput, 1);
    //}
  }

  public void descend(){
    rightClimber_motor.set(ControlMode.PercentOutput, 1);
    leftClimber_motor.set(ControlMode.PercentOutput, -1);
  }

  public void stopLift(){
    rightClimber_motor.set(ControlMode.PercentOutput, 0);
    leftClimber_motor.set(ControlMode.PercentOutput, 0);
  }
}
