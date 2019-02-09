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
  double targetDeg;
   //do we want to create an encoder subclass
  double initialEncoder;
  final double pulseToDegrees = 5.55; //encoder ticks divived by this to get degrees
  final double gearRatio = 0.4; //idk lol
  //one encoder click = 0.28089887640449438202247191011236 degrees
  //one rotation = 1281.6 encoder clicks

  double targetTicks;

  public RotateTurretDegrees(double targetDeg) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis); 
    
    targetDeg = this.targetDeg;
    requires(Robot.turret);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    //Robot.turret.getEncoder().reset();
    
    SmartDashboard.putNumber("initial encoder", Robot.turret.getEncoderValue());    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    initialEncoder = Robot.turret.getEncoderValue();
    double targetDegFin = ((targetDeg + (initialEncoder/pulseToDegrees)) * gearRatio); //variable to store end angle in encoder tick rather than degree

    SmartDashboard.putNumber("initial encoder (tick)", initialEncoder);
    SmartDashboard.putNumber("encoder (tick)", Robot.turret.getEncoderValue());
    SmartDashboard.putNumber("target ((deg+(initEnc(deg)))*GearRat)", targetDegFin); // displays target in degrees from tick value

    //code to spin to the target
    double encoderDegrees = (Robot.turret.getEncoderValue()/pulseToDegrees);

    if(encoderDegrees < targetDegFin){
        //while(encoderDegrees < targetDegFin || !Robot.turret.getLimit1Value() || !Robot.turret.getLimit2Value()) {
          Robot.turret.Rotate(1);
        //}
      } 
      
      else if(encoderDegrees > targetDegFin) {
        //while(encoderDegrees > targetDegFin || !Robot.turret.getLimit1Value() || !Robot.turret.getLimit2Value()) {
          Robot.turret.Rotate(-1); //not sure this works
        //}
      }

      else{
        Robot.turret.Stop();
      }
    }  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Robot.turret.getLimit1Value() || Robot.turret.getLimit2Value()) {
      Robot.turret.Stop();
      SmartDashboard.putString("Ends","Limit");
      return true;
    } 
    else {
      double encoderDegrees = (Robot.turret.getEncoderValue() / pulseToDegrees)*gearRatio;
      //double distanceABS = Math.abs(distance);
      SmartDashboard.putNumber("EndEncoder", encoderDegrees);
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
    //Robot.turret.Stop();
    
   // Robot.turret.getlimitSwitch1().close();
    //Robot.turret.getlimitSwitch2().close();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //Robot.turret.Stop();
    //Robot.turret.getlimitSwitch1().close();
    //Robot.turret.getlimitSwitch2().close();
    }
}