/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.ArmPosition;
import frc.robot.RobotMap.Joint;
import frc.robot.commands.DriveMecanum;
import frc.robot.commands.Auto.DriveOffHab;
import frc.robot.commands.PID.ManualArmPID;
import frc.robot.commands.PID.ManualWristPID;
import frc.robot.commands.arm.TestCommand;
import frc.robot.commands.arm.routines.PositionArm;
import frc.robot.subsystems.ArmAssembly;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pneumatics;

//not sure if we need "Timed Robot", as I believe we used Iterative last year,
//but this is the automatic for a cmd based robot project
public class Robot extends TimedRobot {
  public static OI m_oi;
  public static DriveTrain driveTrain = new DriveTrain();
  public static ArmAssembly armAssembly = new ArmAssembly();
  public static Pneumatics pneumatics = new Pneumatics();
  public static Climber climber = new Climber();
  public Compressor compressor = new Compressor();

  public static double visionHorizontalOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  public static double visionVerticalOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  SendableChooser<Command> testChooser = new SendableChooser<>();

  //runs when robot is initialized
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new DriveMecanum());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    SmartDashboard.putBoolean("Zero Arm/Wrist Encoders", false);
    m_chooser.addOption("Drive Off Hab", new DriveOffHab());
    m_chooser.setDefaultOption("NONE", new TestCommand());
    compressor.setClosedLoopControl(true);
    SmartDashboard.putNumber("Angle", 0);
    
    SmartDashboard.putData("Test mode", testChooser);
    testChooser.setDefaultOption("None", new TestCommand());

    CameraServer.getInstance().startAutomaticCapture();


    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setDouble(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setDouble(1);
  }

  //runs periodically while active
  @Override
  public void robotPeriodic() {
    
    m_chooser.addOption("Arm Angle", new ManualArmPID(SmartDashboard.getNumber("Angle", 90), 2));
    m_chooser.addOption("Hatch Position", new ManualWristPID(SmartDashboard.getNumber("Angle", 90), 2));
    m_chooser.addOption("Default Position", new PositionArm(ArmPosition.DEFAULT));
    m_chooser.addOption("Holding Position", new PositionArm(ArmPosition.HOLDING));

    SmartDashboard.putNumber("BackFinger Angle", Robot.armAssembly.getJointAngle(Joint.BACK_FINGER));
    SmartDashboard.putNumber("FrontFinger Angle", Robot.armAssembly.getJointAngle(Joint.FRONT_FINGER));
    SmartDashboard.putData("Arm PID", Robot.armAssembly.armPID.getPIDController());


  }

  //runs when robot is disabled
  @Override
  public void disabledInit() {
  }

  //runs periodically while robot is disabled
  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  //runs when auto starts, not sure if needed because sandstorm
  @Override
  public void autonomousInit() {
    Robot.armAssembly.setArmEncoder(90);
    Robot.armAssembly.setWristEncoder(90);
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  //called periodically during auto
  @Override
  public void autonomousPeriodic() {
    

    Scheduler.getInstance().run();
  }

  //runs when teleop initializes
  @Override
  public void teleopInit() {
    //stop auto
    Robot.armAssembly.setArmEncoder(90);
    Robot.armAssembly.setWristEncoder(90);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  //runs periodically during teleop
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    SmartDashboard.putNumber("Wrist Speed", Robot.armAssembly.wristMult);
    SmartDashboard.putNumber("Arm Speed", Robot.armAssembly.armMult);
  }

  //runs periodically during test mode
  @Override
  public void testPeriodic() {
    
  }

  public static double getHorizontalOffset(){
    return visionHorizontalOffset;
  }

  public static double getVerticalOffset(){
    return visionVerticalOffset;
  }

  public static void cleanStack(){
    Scheduler.getInstance().removeAll();
  }
}
