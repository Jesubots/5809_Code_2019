/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveMecanum;
import frc.robot.subsystems.ArmAssembly;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pneumatics;

//not sure if we need "Timed Robot", as I believe we used Iterative last year,
//but this is the automatic for a cmd based robot project
public class Robot extends TimedRobot {
  public static OI m_oi;
  public static DriveTrain driveTrain = new DriveTrain();
  public static ArmAssembly armAssembly = new ArmAssembly();
  public static Pneumatics pneumatics = new Pneumatics();

  

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  //runs when robot is initialized
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new DriveMecanum());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
  }

  //runs periodically while active
  @Override
  public void robotPeriodic() {
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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  //runs periodically during teleop
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  //runs periodically during test mode
  @Override
  public void testPeriodic() {
  }
}
