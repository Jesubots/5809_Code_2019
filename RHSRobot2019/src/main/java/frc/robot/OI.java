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
import frc.robot.commands.Climb;
import frc.robot.commands.PID.ConstantLeftIntakePID;
import frc.robot.commands.PID.ConstantRightIntakePID;
import frc.robot.commands.PID.ManualArmPID;
import frc.robot.commands.PID.ManualLeftIntakePID;
import frc.robot.commands.PID.ManualRightIntakePID;
import frc.robot.commands.arm.Fire;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.arm.MoveFingers;
import frc.robot.commands.arm.MoveIntakes;
import frc.robot.commands.arm.MoveWrist;
import frc.robot.commands.arm.RunIntakeEnds;
import frc.robot.commands.arm.routines.PickupBallGround;
import frc.robot.commands.arm.routines.PositionArm;

//controls class
public class OI {
  //controller(s)
  public static final Joystick driverStick = new Joystick(0);
  public static final Joystick buttonPanel = new Joystick(1);
  //arm
  private static double wristAngle = 0;
  private static int armDir = 1;
  //buttons
  //Pickup
  public static JoystickButton hatchGroundPickup_b = new JoystickButton(buttonPanel, 10);
  public static JoystickButton hatchWallPickup_b = new JoystickButton(buttonPanel, 5);
  public static JoystickButton hatchPlace_b = new JoystickButton(buttonPanel, 6);
  public static JoystickButton ballGroundPickup_b = new JoystickButton(buttonPanel, 14);
  //Shooting
  public static JoystickButton aimRocketLow_b = new JoystickButton(buttonPanel, 10);
  public static JoystickButton aimCargo_b = new JoystickButton(buttonPanel, 8);
  public static JoystickButton aimRocketMid_b = new JoystickButton(buttonPanel, 12);
  public static JoystickButton shootRocketLow_b = new JoystickButton(buttonPanel, 9);
  public static JoystickButton shootCargo_b = new JoystickButton(buttonPanel, 7);
  public static JoystickButton shootRocketMid_b = new JoystickButton(buttonPanel, 11);
  //Other
  public static JoystickButton flipArm_b = new JoystickButton(driverStick, 2);
  public static JoystickButton climb_b = new JoystickButton(driverStick, 7);
  public static JoystickButton intakePosition_b = new JoystickButton(driverStick, 1);
  public static JoystickButton armUp_b = new JoystickButton(buttonPanel, 3);
  public static JoystickButton armDown_b = new JoystickButton(buttonPanel, 2);
  public static JoystickButton manualShoot_b = new JoystickButton(driverStick, 5);
  public static JoystickButton test_b = new JoystickButton(buttonPanel, 13);

  public static JoystickButton left_stick = new JoystickButton(buttonPanel, 4);
  public static JoystickButton right_stick = new JoystickButton(buttonPanel, 1);

  private double mult = .5;

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

    OI.hatchWallPickup_b.whenPressed(new PositionArm(ArmPosition.HATCH));

    OI.flipArm_b.whenPressed(new ManualArmPID(-Robot.armAssembly.getArmAngle(), 2));

    OI.manualShoot_b.whenPressed(new Fire(true));

    //if(Robot.armAssembly.getArmPosition() == ArmPosition.BALL_PICKUP)
      OI.ballGroundPickup_b.whenPressed(new PickupBallGround());
    //else 
      //OI.ballGroundPickup_b.whenPressed(new PositionArm(ArmPosition.BALL_PICKUP));

    OI.intakePosition_b.whenPressed(new ManualRightIntakePID(0, 2));
    OI.intakePosition_b.whenPressed(new ManualLeftIntakePID(0, 2));

    OI.hatchPlace_b.whenPressed(new PositionArm(ArmPosition.HOLDING));

    //if(Robot.armAssembly.getArmPosition() == ArmPosition.HATCH)
      //OI.hatchPlace_b.whenPressed(new PlaceHatch()); 
    //else
      //OI.hatchPlace_b.whenPressed(new PositionArm(ArmPosition.HATCH));

    //OI.shootRocketLow_b.whenPressed(new ShootBall(BallTarget.LOW));
    //OI.aimRocketLow_b.whenPressed(new PositionArm(ArmPosition.SHOOT_LOW)); 
    OI.aimRocketMid_b.whenPressed(new PickupBallGround());
    OI.aimRocketLow_b.whenPressed(new PositionArm(ArmPosition.SHOOT_LOW));

    OI.shootRocketLow_b.whileHeld(new MoveFingers(1));
    OI.shootRocketMid_b.whenPressed(new PositionArm(ArmPosition.BALL_PICKUP));

    //OI.shootRocketMid_b.whenPressed(new ShootBall(BallTarget.MID)); 
    //OI.aimRocketMid_b.whenPressed(new PositionArm(ArmPosition.SHOOT_MID)); 
    OI.left_stick.whileHeld(new MoveWrist(-.2));
    OI.right_stick.whileHeld(new MoveWrist(.2));

    OI.aimCargo_b.whenPressed(new PositionArm(ArmPosition.SHOOT_CARGO));
    OI.shootCargo_b.whileHeld(new MoveIntakes(-1));
    OI.shootCargo_b.whileHeld(new RunIntakeEnds(1, -.75, .75));
    //OI.shootCargo_b.whenPressed(new ShootBall(BallTarget.CARGO)); 
    //OI.aimCargo_b.whenPressed(new PositionArm(ArmPosition.SHOOT_CARGO)); 

    OI.climb_b.whenPressed(new Climb());

    OI.armDown_b.whileHeld(new MoveArm(-.25)); 
    OI.armUp_b.whileHeld(new MoveArm(.25));

    OI.test_b.whenPressed(new ConstantRightIntakePID(45f, 10f));
    OI.test_b.whenPressed(new ConstantLeftIntakePID(45f, 10f));
  }

  public static int getArmDir(){
    return armDir;
  }

  public static void setArmDir(int dir){
    armDir = dir;
  }
}
