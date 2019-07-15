/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class XboxDrive extends Command {

  XboxController xbox;

  double x1;
  double x2;
  double y1;
  double y2;

  double leftPow;
  double rightPow;

  public XboxDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time

  @Override
  protected void initialize() {
    xbox = Robot.m_oi.getXbox();
  }

  // Called repeatedly when this Command is scheduled to run

  @Override
  protected void execute() {
    x1 = xbox.getX(Hand.kLeft);
    y1 = xbox.getY(Hand.kLeft);
    x2 = xbox.getX(Hand.kRight);
    y2 = xbox.getY(Hand.kRight);
    if (x1 >= -Robot.driveTrain.deadzone && x1 <= Robot.driveTrain.deadzone) {
      y1 = 0;
    }
    if (x2 >= -Robot.driveTrain.deadzone && x2 <= Robot.driveTrain.deadzone) {
      y2 = 0;
    }
    if (y2 >= -Robot.driveTrain.deadzone && y2 <= Robot.driveTrain.deadzone) {
      y2 = 0;
    }
    if (y1 >= -Robot.driveTrain.deadzone && y1 <= Robot.driveTrain.deadzone) {
      y1 = 0;
    }
    

    rightPow = -(y1 + x1);
    leftPow = (y1 - x1);

    Robot.driveTrain.setMotorPower(0, rightPow);
    Robot.driveTrain.setMotorPower(2, rightPow);

    Robot.driveTrain.setMotorPower(1, leftPow);
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
