/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class RotateTurretDegrees extends Command {
  final double encoderRange = 1;
  double target;
   //do we want to create an encoder subclass
  double initialEncoder;
  final double pulseToDegrees = 5.55; //encoder ticks divived by this to get degrees
  final double gearRatio = 2/5; //idk lol
  //one encoder click = 0.28089887640449438202247191011236 degrees
  //one rotation = 1281.6 encoder clicks

  public RotateTurretDegrees(double target) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    target = this.target;
    requires(Robot.turret);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    //Robot.turret.getEncoder().reset();
    
    SmartDashboard.putNumber("initial encoder", Robot.turret.getEncoderDistance());
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    initialEncoder = Robot.turret.getEncoderDistance();
    target = (target + initialEncoder)*pulseToDegrees*gearRatio;
    SmartDashboard.putNumber("start distance", initialEncoder);
    SmartDashboard.putNumber("encoder", Robot.turret.getEncoderDistance());
    SmartDashboard.putNumber("target", target);
    //SmartDashboard.putNumber("Distance per Pulse", Robot.turret.getEncoder().getDistancePerPulse());
    if(Robot.turret.getEncoderDistance() < target) {
      Robot.turret.Rotate(1);
    } else if(Robot.turret.getEncoderDistance() > target) {
      Robot.turret.Rotate(-1); //not sure this works
    }else{
      Robot.turret.Stop();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(!Robot.turret.getLimit1Value() || !Robot.turret.getLimit2Value()) {
      return true;
    } else {
      double distance = Robot.turret.getEncoderDistance()/pulseToDegrees;
      //double distanceABS = Math.abs(distance);
      SmartDashboard.putNumber("EndEncoder", distance);
      if(distance >= target - encoderRange && distance <= target + encoderRange) {//&& distanceABS <= target + encoderRange
        return true;
      } else {
        return false;
      }
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.turret.Stop();
    
   // Robot.turret.getlimitSwitch1().close();
    //Robot.turret.getlimitSwitch2().close();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.turret.Stop();
    //Robot.turret.getlimitSwitch1().close();
    //Robot.turret.getlimitSwitch2().close();
    }
}