/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*; //imports all commands

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  Joystick stick = new Joystick(1);
  Joystick stick2 = new Joystick(2);

  Button button_1_1 = new JoystickButton(stick, 1);
  Button button_1_2 = new JoystickButton(stick, 2);
  Button button_1_3 = new JoystickButton(stick, 3);
  Button button_1_4 = new JoystickButton(stick, 4);
  Button button_1_5 = new JoystickButton(stick, 5);
  Button button_1_6 = new JoystickButton(stick, 6);
  Button button_1_7 = new JoystickButton(stick, 7);
  Button button_1_9 = new JoystickButton(stick, 9);
  Button button_1_10 = new JoystickButton(stick, 10);
  Button button_1_11 = new JoystickButton(stick, 11);
  Button button_1_12 = new JoystickButton(stick, 12);

  Button button_2_1 = new JoystickButton(stick2, 1);
  Button button_2_2 = new JoystickButton(stick2, 2);
  Button button_2_3 = new JoystickButton(stick2, 3);
  Button button_2_4 = new JoystickButton(stick2, 4);
  Button button_2_5 = new JoystickButton(stick2, 5);
  Button button_2_6 = new JoystickButton(stick2, 6);
  Button button_2_7 = new JoystickButton(stick2, 7);
  Button button_2_9 = new JoystickButton(stick2, 9);
  Button button_2_10 = new JoystickButton(stick2, 10);
  Button button_2_11 = new JoystickButton(stick2, 11);
  Button button_2_12 = new JoystickButton(stick2, 12);
  

  XboxController xbox = new XboxController(0);

  Button xbox1 = new JoystickButton(xbox, 1);
  //Button xbox1 = new JoystickButton(xbox, 1);

  public OI() {

    button_1_1.whenPressed(new TestSol());
    //button_1_2.whenPressed(new FireSol());
    button_1_3.whileHeld(new RotateTurret(1)); //forwards soon i hope it is clockwise rn
    //button_1_4.whenPressed(new RotateTurretDegrees(180));
    button_1_5.whileHeld(new RotateTurret(-1)); //backwards soon counterclockwise rn
    //button_1_6.whenPressed(new RotateTurretDegrees(-180));
    //button_1_5.whenPressed(new SwitchLight());
    button_1_6.whenPressed(new FireEndSol());
    button_1_7.whenPressed(new CameraSwitch());
    xbox1.whenPressed(new FireSol());
    button_1_2.whileHeld(new AutoAlign());
    button_1_12.whenPressed(new DriveSwitch());
    button_1_11.whenPressed(new Rumble());

    button_2_1.whenPressed(new ResetNavx());
    //button_1_9.whileHeld(new DriveStraightSub("forwards"));
    //button_1_9.whileHeld(new DriveStraightSub("backwards"));
  }

  //// CREATING BUTTON // One type of button_1_ is a joystick button_1_ which is any button_1_ on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button_1_
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button_1_ = new JoystickButton(stick, button_1_Number);

  // There are a few additional built in button_1_s you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button_1_, it's trivial to bind it to a button_1_ in one of
  // three ways:

  // Start the command when the button_1_ is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button_1_.whenPressed(new ExampleCommand());

  // Run the command while the button_1_ is being held down and interrupt it once
  // the button_1_ is released.
  // button_1_.whileHeld(new ExampleCommand());

  // Start the command when the button_1_ is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button_1_.whenReleased(new ExampleCommand());

  public Joystick getJoystick() {
    return stick;
  }

  public XboxController getXbox() {
    return xbox;
  }

  public Joystick getJoystick2() {
	  return stick2;
  }

}
