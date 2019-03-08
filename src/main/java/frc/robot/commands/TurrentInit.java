/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TurrentInit extends Command {
  boolean TurrentInitExecute = false;
  public TurrentInit() {
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    
    //requires(Robot.turret);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    
    Robot.turret.Rotate(-1);
    
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //reset encoder here
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
