/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TurnDegreesInPlace extends Command {
  public TurnDegreesInPlace() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  Robot.driveTrain.getBLEncoder();
  Robot.driveTrain.getBREncoder();
  Robot.driveTrain.getFLEncoder();
  Robot.driveTrain.getFREncoder();
  }




  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.setMotorPower(0, -1);
    Robot.driveTrain.setMotorPower(1, -1); 
    Robot.driveTrain.setMotorPower(2, 1);
    Robot.driveTrain.setMotorPower(3, 1);

  
    


  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Robot.driveTrain.getFLEncoder()< 5){    
      return true;      
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
