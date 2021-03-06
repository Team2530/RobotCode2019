/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Drive2 extends Command {
  public Drive2() {
    //Robot robot = new Robot(); //idek
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    requires(Robot.turret);
    requires(Robot.intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Joystick stick = Robot.m_oi.getJoystick();
    Joystick stick2 = Robot.m_oi.getJoystick2();
    XboxController xbox = Robot.m_oi.getXbox();
    Robot.driveTrain.Drive2(stick, stick2);
    Robot.turret.Rotate(xbox.getX(Hand.kLeft));
    Robot.intake.Intake(xbox.getPOV());
    Robot.intake.UpAndDown(xbox.getY(Hand.kRight));
    //SmartDashboard.putNumber("Dpad", xbox.getPOV());
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
