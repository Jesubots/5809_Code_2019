/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.RobotMap;
import frc.robot.RobotMap.Joint;
import frc.robot.subsystems.PID.PotPID;

/**
 * Add your docs here.
 */
public class ArmAssembly extends Subsystem {
  public VictorSP topFinger_motor = new VictorSP(RobotMap.ArmAssemblyMap.topFinger_motor_port);
  public VictorSP bottomFinger_motor = new VictorSP(RobotMap.ArmAssemblyMap.bottomFinger_motor_port);
  public VictorSP wrist_motor = new VictorSP(RobotMap.ArmAssemblyMap.wrist_motor_port);
  public VictorSP armMaster_motor = new VictorSP(RobotMap.ArmAssemblyMap.armMaster_motor_port);
  public VictorSP armFollower_motor = new VictorSP(RobotMap.ArmAssemblyMap.armFollower_motor_port);
  public VictorSP leftIntakeEnd_motor = new VictorSP(RobotMap.ArmAssemblyMap.leftIntakeEnd_motor_port);
  public VictorSP leftIntakeArm_motor = new VictorSP(RobotMap.ArmAssemblyMap.leftIntakeArm_motor_port);
  public VictorSP rightIntakeEnd_motor = new VictorSP(RobotMap.ArmAssemblyMap.rightIntakeEnd_motor_port);
  public VictorSP rightIntakeArm_motor = new VictorSP(RobotMap.ArmAssemblyMap.rightIntakeArm_motor_port);

  public Potentiometer topFinger_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.topFinger_pot_port, 360, 30);
  public Potentiometer bottomFinger_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.bottomFinger_pot_port, 360, 30);
  public Potentiometer wrist_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.wrist_pot_port, 360, 30);
  public Potentiometer arm_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.arm_pot_port, 360, 30);
  public Potentiometer leftIntakeArm_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.leftIntakeArm_pot_port, 360, 30);
  public Potentiometer rightIntakeArm_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.rightIntakeArm_pot_port, 360, 30);

  public PotPID potPID = new PotPID();

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void JointToAnglePID(double angle, Joint joint){
    potPID.setJoint(joint);
    potPID.setSetpoint(angle);
    potPID.enable();
  }

  public void moveJoint(VictorSP victor, double output){
    victor.set(output);
  }

  public void moveJoint(VictorSPX victor, double output){
    victor.set(ControlMode.PercentOutput, output);
  }

  public void moveJoint(Spark spark, double output){
    spark.set(output);
  }

  public void moveJoint(WPI_TalonSRX talon, double output){
    talon.set(output);
  }
  
  public double getPotAngle(Joint joint){
    if(joint == Joint.kARM){
      return arm_pot.get();
    } else if(joint == Joint.kLEFT_INTAKE){
      return leftIntakeArm_pot.get();
    } else if(joint == Joint.kRIGHT_INTAKE){
      return rightIntakeArm_pot.get();
    } else if(joint == Joint.kTOP_FINGER){
      return topFinger_pot.get();
    } else if(joint == Joint.kBOTTOM_FINGER){
      return bottomFinger_pot.get();
    } else if(joint == Joint.kWRIST){
      return wrist_pot.get();
    } else {
      System.out.println("Cannot get potentiometer angle. No joint selected.");
      return 0;
    }
  }
}
