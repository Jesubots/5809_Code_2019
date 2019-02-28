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
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.RobotMap;
import frc.robot.RobotMap.Joint;
import frc.robot.subsystems.PID.PotPID;

/**
 * Add your docs here.
 */
public class ArmAssembly extends Subsystem {
  public VictorSPX topFinger_motor = new VictorSPX(RobotMap.ArmAssemblyMap.topFinger_motor_port);
  public VictorSPX bottomFinger_motor = new VictorSPX(RobotMap.ArmAssemblyMap.bottomFinger_motor_port);
  public VictorSPX wrist_motor = new VictorSPX(RobotMap.ArmAssemblyMap.wrist_motor_port);
  public WPI_TalonSRX armMaster_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.armMaster_motor_port);
  public WPI_TalonSRX armFollower_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.armFollower_motor_port);
  public WPI_TalonSRX leftIntakeEnd_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.leftIntakeEnd_motor_port);
  public WPI_TalonSRX leftIntakeArm_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.leftIntakeArm_motor_port);
  public VictorSPX rightIntakeEnd_motor = new VictorSPX(RobotMap.ArmAssemblyMap.rightIntakeEnd_motor_port);
  public VictorSPX rightIntakeArm_motor = new VictorSPX(RobotMap.ArmAssemblyMap.rightIntakeArm_motor_port);

  public Potentiometer topFinger_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.topFinger_pot_port, 360, 30);
  public Potentiometer bottomFinger_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.bottomFinger_pot_port, 360, 30);
  public Potentiometer wrist_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.wrist_pot_port, 360, 30);
  public Potentiometer intake_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.intake_pot_port, 360, 30);

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

  public void moveJoint(VictorSPX victor, double output){
    victor.set(ControlMode.PercentOutput, output);
  }

  public void moveJoint(WPI_TalonSRX talon, double output){
    talon.set(output);
  }
  
  public double getPotAngle(Joint joint){
    if(joint == Joint.kARM){
      return 0; //RETURN ARM ENCODER
    } else if(joint == Joint.kINTAKE){
      return intake_pot.get();
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
