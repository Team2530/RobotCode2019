/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class RotateTurretDegrees extends Command {
  final double encoderRange = 5;
  double targetDeg;
   //do we want to create an encoder subclass
  double initialEncoder;
  final double pulseToDegrees = 5.55; //encoder ticks divived by this to get degrees
  final double gearRatio = 42/56; //idk lol
  //42/56
  //one encoder click = 0.28089887640449438202247191011236 degrees
  //one rotation = 1281.6 encoder clicks
  double targetDegFin;
  double targetTicks;
  double encoderDegrees;

  public RotateTurretDegrees(double target) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis); 
    
    targetDeg = target;
    // requires(Robot.turret);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    //Robot.turret.getEncoder().reset();
    SmartDashboard.putString("Ends","");
    SmartDashboard.putNumber("initial encoder", Robot.turret.getEncoderValue());    
    initialEncoder = Robot.turret.getEncoderValue();
    targetDegFin = (targetDeg + ((initialEncoder/pulseToDegrees)* gearRatio));
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    SmartDashboard.putNumber("initial encoder (tick)", initialEncoder);
    SmartDashboard.putNumber("encoder", Robot.turret.getEncoderValue());
    SmartDashboard.putNumber("target (deg)", targetDeg); 
    SmartDashboard.putNumber("targetDegFin", targetDegFin);
    //code to spin to the target
    encoderDegrees = ((Robot.turret.getEncoderValue()/pulseToDegrees)*gearRatio);
    SmartDashboard.putNumber("encoder (deg)", encoderDegrees);



    if((encoderDegrees < targetDegFin) /*&& (encoderDegrees <= targetDegFin - encoderRange)*/){
      Robot.turret.Rotate(.5);
    } else if((encoderDegrees > targetDegFin) /*&& (encoderDegrees >= targetDegFin + encoderRange)*/){
      Robot.turret.Rotate(-.5);
    }
  }  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Robot.turret.getLimit1Value() || Robot.turret.getLimit2Value()) { // Safety block to make sure it stops at limit switches

      SmartDashboard.putString("Ends","Limit");
      return true;
    } else {
      // double encoderDegrees = (Robot.turret.getEncoderValue() / pulseToDegrees)*gearRatio;
      //double distanceABS = Math.abs(distance);
      // SmartDashboard.putNumber("CurrentEncoderDeg", encoderDegrees);
      SmartDashboard.putBoolean("Is Done", (encoderDegrees >= targetDeg - encoderRange) && (encoderDegrees <= targetDeg + encoderRange));


      if((encoderDegrees >= targetDeg - encoderRange) && (encoderDegrees <= targetDeg + encoderRange)) {//&& distanceABS <= target + encoderRange
        SmartDashboard.putString("Ends","Encoder");
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