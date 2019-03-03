/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ArmPosition;
import frc.robot.RobotMap.Joint;
import frc.robot.commands.arm.MoveArm;
import frc.robot.subsystems.PID.PotPID;

/**
 * Add your docs here.
 */
public class ArmAssembly extends Subsystem {
  public VictorSPX topFinger_motor = new VictorSPX(RobotMap.ArmAssemblyMap.topFinger_motor_port);
  public VictorSPX bottomFinger_motor = new VictorSPX(RobotMap.ArmAssemblyMap.bottomFinger_motor_port);
  public WPI_TalonSRX wrist_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.wrist_motor_port);
  public WPI_TalonSRX armMaster_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.armMaster_motor_port);
  public WPI_TalonSRX armFollower_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.armFollower_motor_port);
  public WPI_TalonSRX leftIntakeEnd_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.leftIntakeEnd_motor_port);
  public WPI_TalonSRX leftIntakeArm_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.leftIntakeArm_motor_port);
  public VictorSPX rightIntakeEnd_motor = new VictorSPX(RobotMap.ArmAssemblyMap.rightIntakeEnd_motor_port);
  public VictorSPX rightIntakeArm_motor = new VictorSPX(RobotMap.ArmAssemblyMap.rightIntakeArm_motor_port);

  public Potentiometer topFinger_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.topFinger_pot_port, 360, 30);
  public Potentiometer bottomFinger_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.bottomFinger_pot_port, 360, 30);
  public Potentiometer leftIntake_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.leftIntake_pot_port, 360, 30);
  public Potentiometer rightIntake_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.rightIntake_pot_port, 360, 30);

  private ArmPosition armPosition = ArmPosition.kDEFAULT;
  public PotPID potPID = new PotPID();

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new MoveArm());
  }

  public ArmPosition getArmPosition(){
    return armPosition;
  }

  public void setArmPosition(ArmPosition position){
    armPosition = position;
  }

  public void StartPotPID(double angle, Joint joint){
    potPID.setJoint(joint);
    potPID.setSetpoint(angle);
    potPID.enable();
  }

  public void StopPotPID(Joint joint){
    potPID.disable();
    cancelMotors(joint);
  }

  public void moveJoint(VictorSPX victor, double output){
    victor.set(ControlMode.PercentOutput, output);
  }

  public void moveJoint(WPI_TalonSRX talon, double output){
    talon.set(output);
  }

  public double getArmAngle(){
    double angle = 0.0;
    armMaster_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    double encRaw = armMaster_motor.getSelectedSensorPosition();
    angle = encRaw*(360/4096);
    angle += 30.96;
    return angle;
  }

  public double getWristAngle(){
    double angle = 0.0;
    wrist_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    double encRaw = armMaster_motor.getSelectedSensorPosition();
    angle = encRaw*(360/4096);
    return angle;
  }
  
  public double getPotAngle(Joint joint){
    if(joint == Joint.kARM){
      return getArmAngle();
    } else if(joint == Joint.kINTAKE){
      return ((leftIntake_pot.get() + rightIntake_pot.get()) / 2);
    } else if(joint == Joint.kTOP_FINGER){
      return topFinger_pot.get();
    } else if(joint == Joint.kBOTTOM_FINGER){
      return bottomFinger_pot.get();
    } else if(joint == Joint.kWRIST){
      return getWristAngle();
    } else {
      System.out.println("Cannot get potentiometer angle. No joint selected.");
      return 0;
    }
  }

  private void cancelMotors(Joint joint){
    //all this does is turn off whichever motor is being used by the PID
    if(joint == Joint.kARM){
      moveJoint(armMaster_motor, 0);
    } else if(joint == Joint.kINTAKE){
      moveJoint(rightIntakeArm_motor, 0);
    } else if(joint == Joint.kTOP_FINGER){
      moveJoint(topFinger_motor, 0);
    } else if(joint == Joint.kBOTTOM_FINGER){
      moveJoint(bottomFinger_motor, 0);
    } else if(joint == Joint.kWRIST){
      moveJoint(wrist_motor, 0);
    } else {
      System.out.println("Cannot cancel motors. No motor selected.");
    }
  }
}
