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

  @Override
  public void initDefaultCommand() {
    //default command
  }
}
