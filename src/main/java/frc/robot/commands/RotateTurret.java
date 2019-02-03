/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class RotateTurret extends Command {

  DigitalInput limitSwitch1;
  DigitalInput limitSwitch2;

  public RotateTurret() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.turret);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    limitSwitch1 = new DigitalInput(6);
    limitSwitch2 = new DigitalInput(7);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putBoolean("LimitSwitch1", limitSwitch1.get());
    SmartDashboard.putBoolean("LimitSwitch2", limitSwitch2.get());
    Robot.turret.Rotate(1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(limitSwitch1.get() == false || limitSwitch2.get() == false) {
      return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.turret.Stop();
    limitSwitch1.free();
    limitSwitch2.free();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.turret.Stop();
    limitSwitch1.free();
    limitSwitch2.free();
  }
}
