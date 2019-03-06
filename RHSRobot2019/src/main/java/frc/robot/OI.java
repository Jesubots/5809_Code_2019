/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.ArmPosition;
import frc.robot.RobotMap.BallTarget;
import frc.robot.RobotMap.Joint;
import frc.robot.commands.Climb;
import frc.robot.commands.PID.JointToAngle;
import frc.robot.commands.arm.FlipArm;
import frc.robot.commands.arm.IntakeBall;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.arm.routines.PickupBallGround;
import frc.robot.commands.arm.routines.PickupHatch;
import frc.robot.commands.arm.routines.PlaceHatch;
import frc.robot.commands.arm.routines.PositionArm;
import frc.robot.commands.arm.routines.ShootBall;

//controls class
public class OI {
  //controller(s)
  public static final Joystick driverStick = new Joystick(0);
  public static final Joystick buttonPanel = new Joystick(1);
  //arm
  private static double wristAngle = 0;
  private static int armDir = 1;
  //buttons, denoted by _b
  public static JoystickButton hatchGroundPickup_b = new JoystickButton(buttonPanel, 10);
  public static JoystickButton hatchWallPickup_b = new JoystickButton(buttonPanel, 1);
  public static JoystickButton hatchPlace_b = new JoystickButton(buttonPanel, 2);
  public static JoystickButton ballGroundPickup_b = new JoystickButton(buttonPanel, 3);
  public static JoystickButton shootRocketLow_b = new JoystickButton(buttonPanel, 4);
  public static JoystickButton shootCargo_b = new JoystickButton(buttonPanel, 5);
  public static JoystickButton shootRocketMid_b = new JoystickButton(buttonPanel, 6);
  public static JoystickButton flipArm_b = new JoystickButton(buttonPanel, 9);
  public static JoystickButton climb_b = new JoystickButton(driverStick, 5);
  public static JoystickButton intakePosition_b = new JoystickButton(driverStick, 1);
  public static JoystickButton armUp_b = new JoystickButton(buttonPanel, 7);
  public static JoystickButton armDown_b = new JoystickButton(buttonPanel, 8);

  public OI(){
    initButtons();
  }

  public void dashboardEncoderReset(){
    if(SmartDashboard.getBoolean("Zero Arm/Wrist Encoders", false)){
      Robot.armAssembly.resetJointEncoders();
    }
  }
  
  //methods
  public void initButtons(){
    //"SW" indicates "should work", "NW" indicates "needs work"

    OI.flipArm_b.whenPressed(new FlipArm()); //SW
    OI.flipArm_b.whenPressed(new JointToAngle(Joint.kARM, Robot.armAssembly.getArmAngle() * -1, 2)); //SW

    if(Robot.armAssembly.getArmPosition() == ArmPosition.kBALL_PICKUP){
      OI.ballGroundPickup_b.whenPressed(new PickupBallGround()); //NW
    } else {
      OI.ballGroundPickup_b.whenPressed(new PositionArm(ArmPosition.kBALL_PICKUP));
      OI.ballGroundPickup_b.whenPressed(new IntakeBall());
    }

    OI.intakePosition_b.whenPressed(new JointToAngle(Joint.kINTAKE, RobotMap.MAXIMUM_INTAKE_ANGLE, 2)); //SW


    if(Robot.armAssembly.getArmPosition() == ArmPosition.kHATCH)
      OI.hatchPlace_b.whenPressed(new PickupHatch()); //NW
    else
      OI.hatchPlace_b.whenPressed(new PositionArm(ArmPosition.kHATCH));

    if(Robot.armAssembly.getArmPosition() == ArmPosition.kHATCH)
      OI.hatchPlace_b.whenPressed(new PlaceHatch()); //NW
    else
      OI.hatchPlace_b.whenPressed(new PositionArm(ArmPosition.kHATCH));

    if(Robot.armAssembly.getArmPosition() == ArmPosition.kSHOOT_LOW)
      OI.shootRocketLow_b.whenPressed(new ShootBall(BallTarget.kLOW)); //NW
    else
      OI.shootRocketLow_b.whenPressed(new PositionArm(ArmPosition.kSHOOT_LOW));

    if(Robot.armAssembly.getArmPosition() == ArmPosition.kSHOOT_MID)
      OI.shootRocketMid_b.whenPressed(new ShootBall(BallTarget.kMID)); //NW
    else
      OI.shootRocketMid_b.whenPressed(new PositionArm(ArmPosition.kSHOOT_MID)); //NW

    if(Robot.armAssembly.getArmPosition() == ArmPosition.kSHOOT_CARGO)
      OI.shootRocketMid_b.whenPressed(new ShootBall(BallTarget.kCARGO)); //NW
    else
      OI.shootRocketMid_b.whenPressed(new PositionArm(ArmPosition.kSHOOT_CARGO)); //NW

    OI.climb_b.whenPressed(new Climb()); //NW

    OI.armDown_b.whileHeld(new MoveArm(-.25));
    OI.armDown_b.whileHeld(new MoveArm(.25));
  }

  public static int getArmDir(){
    return armDir;
  }

  public static void setArmDir(int dir){
    armDir = dir;
  }
}
