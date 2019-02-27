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
//import frc.robot.subsystems.CameraSub;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  Joystick stick = new Joystick(1);
  Joystick stick2 = new Joystick(2);

  Button button1 = new JoystickButton(stick, 1);
  Button button2 = new JoystickButton(stick, 2);
  Button button3 = new JoystickButton(stick, 3);
  Button button4 = new JoystickButton(stick, 4);
  Button button5 = new JoystickButton(stick, 5);
  Button button6 = new JoystickButton(stick, 6);
  Button button7 = new JoystickButton(stick, 7);
  Button button9 = new JoystickButton(stick, 9);
  Button button10 = new JoystickButton(stick, 10);
  

  XboxController xbox = new XboxController(0);

  Button xbox1 = new JoystickButton(xbox, 1);
  //Button xbox1 = new JoystickButton(xbox, 1);

  public OI() {

    button1.whenPressed(new TestSol());
    //button2.whenPressed(new FireSol());
    //button3.whileHeld(new RotateTurret(1)); //forwards soon i hope it is clockwise rn
    //button4.whenPressed(new RotateTurretDegrees(180));
    //button5.whileHeld(new RotateTurret(-1)); //backwards soon counterclockwise rn
    //button6.whenPressed(new RotateTurretDegrees(-180));
    //button5.whenPressed(new SwitchLight());
    button6.whenPressed(new FireEndSol());
    button7.whenPressed(new CameraSub());
    xbox1.whenPressed(new FireSol());
    button2.whileHeld(new AutoAlign());
    button9.whileHeld(new DriveStraightSub(1));
    button10.whileHeld(new DriveStraightSub(-1));
  }

  //// CREATING BUTTON // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

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
