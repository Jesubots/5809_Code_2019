/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

  /***** how it works *****\
  * take inputs: X, Y, and Twist
  * The X input will become the X component of the vector.
  * The Y input will become the Y component.
  * The Twist input will turn the robot, as a tank drive turns.
  * The Y component is as simple as passing that value to all motors...
  * The X component is weird...
  * The X input has to be passed normally to the top left and bottom right motors, 
  * and opposite to the top right and bottom left motors.
  * You add all these separate inputs, and that's really it.
  * Vectors aren't *that* necessary, but they look real nice, so I'll use them.
  \************************/

public class MecanumDrive extends Command {
  public MecanumDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
