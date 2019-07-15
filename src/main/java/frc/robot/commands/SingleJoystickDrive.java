/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class SingleJoystickDrive extends Command {
  
  Joystick stick;
  
  double y1;
  
  double x1;
  double z1;

  double leftPow;
  double rightPow;
  
  public SingleJoystickDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time



  @Override
  protected void initialize() {
    stick =Robot.m_oi.getJoystick();
  }

  // Called repeatedly when this Command is scheduled to run
  
  @Override
  protected void execute() {
    x1 = Robot.driveTrain.getDriveDirection()*stick.getX();
    y1 = Robot.driveTrain.getDriveDirection()*stick.getY();
    z1 = stick.getZ();
    if (x1 >= -Robot.driveTrain.deadzone && x1 <= Robot.driveTrain.deadzone) {
      x1 = 0;
    } 
    if (y1 >= -Robot.driveTrain.deadzone && y1 <= Robot.driveTrain.deadzone) {
      y1 = 0;
    } 
    if (Math.abs(z1) <= 0.3) {
      z1 = 0;
    }

    SmartDashboard.putNumber("x", x1);
    SmartDashboard.putNumber("y", y1);
    SmartDashboard.putNumber("z", z1);

    rightPow = (y1 + z1);
    leftPow = (y1 - z1);
  
    Robot.driveTrain.setMotorPower(0,rightPow);
    Robot.driveTrain.setMotorPower(2,rightPow);

    Robot.driveTrain.setMotorPower(1,leftPow);
    Robot.driveTrain.setMotorPower(3, leftPow);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.Stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.Stop();
  }
}
