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

  int power;

  public RotateTurret(int speed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    power = speed;
    requires(Robot.turret);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putBoolean("LimitSwitch1", Robot.turret.getlimitSwitch1().get());
    SmartDashboard.putBoolean("LimitSwitch2", Robot.turret.getlimitSwitch2().get());
    SmartDashboard.putNumber("encoder", Robot.turret.getEncoderDistance());
    if(Robot.turret.getLimit1Value() && power > 0) { //false is closed on NO, but closed is true on NC
      
    } else if(Robot.turret.getLimit2Value() && power < 0) { //false is closed on ON, but closed is true on NC
      
    } else {
      Robot.turret.Rotate(power);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Robot.turret.getLimit1Value()&& power > 0) { //false is closed on NO, but closed is true on NC
      return true;
    } else if(Robot.turret.getLimit2Value() && power < 0) { //false is closed on ON, but closed is true on NC
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
