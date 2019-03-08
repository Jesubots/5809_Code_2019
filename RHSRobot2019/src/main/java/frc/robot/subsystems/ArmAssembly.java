/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ArmPosition;
import frc.robot.RobotMap.Joint;
import frc.robot.subsystems.PID.EncoderPID;
import frc.robot.subsystems.PID.PotPID;

/**
 * Add your docs here.
 */
public class ArmAssembly extends Subsystem {
  public VictorSPX topFinger_motor = new VictorSPX(RobotMap.ArmAssemblyMap.topFinger_motor_port);
  public VictorSPX bottomFinger_motor = new VictorSPX(RobotMap.ArmAssemblyMap.bottomFinger_motor_port);
  public WPI_TalonSRX wrist_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.wrist_motor_port);
  public WPI_TalonSRX armFront_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.armFront_motor_port);
  public WPI_TalonSRX armBack_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.armBack_motor_port);
  public WPI_TalonSRX leftIntakeEnd_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.leftIntakeEnd_motor_port);
  public WPI_TalonSRX leftIntakeArm_motor = new WPI_TalonSRX(RobotMap.ArmAssemblyMap.leftIntakeArm_motor_port);
  public VictorSPX rightIntakeEnd_motor = new VictorSPX(RobotMap.ArmAssemblyMap.rightIntakeEnd_motor_port);
  public VictorSPX rightIntakeArm_motor = new VictorSPX(RobotMap.ArmAssemblyMap.rightIntakeArm_motor_port);

  public Potentiometer topFinger_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.topFinger_pot_port, 3600, -2448);
  public Potentiometer bottomFinger_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.bottomFinger_pot_port, 3600, -2190);
  public Potentiometer leftIntake_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.leftIntake_pot_port, 3600, -285);
  public Potentiometer rightIntake_pot = new AnalogPotentiometer(RobotMap.ArmAssemblyMap.rightIntake_pot_port, 3600, -556);

  private ArmPosition armPosition = ArmPosition.kDEFAULT;
  public PotPID frontFingerPID = new PotPID();
  public PotPID backFingerPID = new PotPID();
  public PotPID intakePID = new PotPID();
  public EncoderPID armPID = new EncoderPID();
  public EncoderPID wristPID = new EncoderPID();

  public DigitalInput limitSwitch = new DigitalInput(RobotMap.limitSwitch_port);
  
  public ArmAssembly(){
    
  }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new MoveArm());
  }

  public ArmPosition getArmPosition(){
    return armPosition;
  }

  public void setArmPosition(ArmPosition position){
    armPosition = position;
  }

  public void setArmEncoder(double angle){
    armFront_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    armFront_motor.setSelectedSensorPosition((int)(angle/(360f/4096f)));
  }

  public void setWristEncoder(double angle){
    wrist_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    wrist_motor.setSelectedSensorPosition((int)(angle/(360f/4096f)));
  }

  public void StartJointPID(double angle, Joint joint){
    if(joint == Joint.kARM){
      armPID.setJoint(joint);
      armPID.setSetpoint(angle);
      armPID.enable();
    } else if(joint == Joint.kBOTTOM_FINGER){
      backFingerPID.setJoint(joint);
      backFingerPID.setSetpoint(angle);
      backFingerPID.enable();
    } else if(joint == Joint.kINTAKE){
      intakePID.setJoint(joint);
      intakePID.setSetpoint(angle);
      intakePID.enable();
    } else if(joint == Joint.kTOP_FINGER){
      frontFingerPID.setJoint(joint);
      frontFingerPID.setSetpoint(angle);
      frontFingerPID.enable();
    } else if(joint == Joint.kWRIST){
      wristPID.setJoint(joint);
      wristPID.setSetpoint(angle);
      wristPID.enable();
    }
    
  }

  public void StopJointPID(Joint joint){
    if(joint == Joint.kARM){
      Robot.pneumatics.brakeOn();
      armPID.disable();
    } else if(joint == Joint.kBOTTOM_FINGER){
      backFingerPID.disable();
    } else if(joint == Joint.kINTAKE){
      intakePID.disable();
    } else if(joint == Joint.kTOP_FINGER){
      frontFingerPID.disable();
    } else if(joint == Joint.kWRIST){
      wristPID.disable();
    }
  }

  public void moveJoint(VictorSPX victor, double output){
    victor.set(ControlMode.PercentOutput, output);
  }

  public void moveJoint(WPI_TalonSRX talon, double output){
    talon.set(ControlMode.PercentOutput, output);
  }

  public double getArmAngle(){
    double angle = 0.0;
    armFront_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    double encRaw = armFront_motor.getSelectedSensorPosition();
    angle = ((double)encRaw)*(360f/4096f);
    return angle;
  }

  public double getWristAngle(){
    double angle = 0.0;
    wrist_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    double encRaw = armFront_motor.getSelectedSensorPosition();
    angle = ((double)encRaw)*(360f/4096f);
    if(angle < 0){
      angle = 0;
    }
    return angle;
  }

  public void resetJointEncoders(){
    wrist_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    wrist_motor.setSelectedSensorPosition(0);
    armFront_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    armFront_motor.setSelectedSensorPosition(0);

  }
  
  public double getPotAngle(Joint joint){
    if(joint == Joint.kARM){
      return getArmAngle();
    } else if(joint == Joint.kINTAKE){
      return ((rightIntake_pot.get() - leftIntake_pot.get()) / 2);
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
      moveJoint(armFront_motor, 0);
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
