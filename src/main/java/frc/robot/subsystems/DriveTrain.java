/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.TestDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  VictorSP motor0 = new VictorSP(0);
  VictorSP motor2 = new VictorSP(2);
    
  VictorSPX motor1 = new VictorSPX(0); //id 0
  VictorSPX motor3 = new VictorSPX(1); //id 1

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new TestDrive());
  }

  public void Drive(Joystick stick) {
    motor0.set(stick.getY());
    motor2.set(stick.getY());

    motor1.set(ControlMode.PercentOutput, stick.getX());
    motor3.set(ControlMode.PercentOutput, stick.getX());
  }

  public void XboxDrive(XboxController xbox) {
    motor0.set(xbox.getY());
    motor2.set(xbox.getY());

    motor1.set(ControlMode.PercentOutput,xbox.getX());
    motor3.set(ControlMode.PercentOutput, xbox.getX());
  }

  public void Stop() {
    motor0.set(0);
    motor2.set(0);
    motor1.set(ControlMode.PercentOutput, 0);
    motor3.set(ControlMode.PercentOutput, 0);
  }
}
