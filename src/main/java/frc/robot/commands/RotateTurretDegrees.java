/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class RotateTurretDegrees extends Command {
  final double encoderRange = 1;
  double target;
  Encoder encoder = new Encoder(8, 9);
  double initialEncoder;
  final double pulseToDegrees = 5.55;
  //one encoder click = 0.28089887640449438202247191011236 degrees
  //one rotation = 1281.6 encoder clicks

  public RotateTurretDegrees(double TargetDegrees) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    target = TargetDegrees;
    requires(Robot.turret);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    encoder.reset();
    initialEncoder = encoder.getDistance();
    target = target + initialEncoder;
    SmartDashboard.putNumber("start distance", initialEncoder);
    SmartDashboard.putNumber("encoder", encoder.getDistance());
    SmartDashboard.putNumber("target", target);
    encoder.setDistancePerPulse((double) 1);
    SmartDashboard.putNumber("Distance per Pulse", encoder.getDistancePerPulse());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.turret.Rotate(-1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double distance = encoder.getDistance()/pulseToDegrees;
    double distanceABS = Math.abs(distance);
    SmartDashboard.putNumber("encoder", distance);
    if(distanceABS >= target - encoderRange) {//&& distanceABS <= target + encoderRange
      return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.turret.Stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.turret.Stop();
  }
}
