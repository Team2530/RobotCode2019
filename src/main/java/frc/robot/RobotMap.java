/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.VictorSP;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static final double driveDelta = 1;//TODO needs to be calibrated
  public static VictorSP motor0 = new VictorSP(0); // fakeid = idk
  public static VictorSP motor2 = new VictorSP(2); // fakeid = idk

  public static VictorSPX motor1 = new VictorSPX(0); // id 0 //fakeid = 2
  public static VictorSPX motor3 = new VictorSPX(1); // id 1 //fakeid = 4

  // these are for joystick

  public static final double deadzone = 0.1; // deadzone stuff
  public static final double zDeadzone = 0.3; // deadzone for z axis turns
  public static double powerfactor = 1; // power multiplier
  public static int driveDirection = 1; // used for multidirectional drive
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
