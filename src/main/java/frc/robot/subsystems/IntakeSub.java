/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class IntakeSub extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  VictorSPX motorLift = new VictorSPX(6);
  VictorSPX motorIntake = new VictorSPX(7);
  DigitalInput limitSwitch3 = new DigitalInput(5);
  DigitalInput limitSwitch4 = new DigitalInput(6);
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void UpAndDown(double speed) { //xbox joystick input
    SmartDashboard.putBoolean("LimitSwitch3", limitSwitch3.get());
    SmartDashboard.putBoolean("LimitSwitch4", limitSwitch4.get());
    if(limitSwitch3.get() && speed > 0) { //false is closed on NO, but closed is true on NC
      motorLift.set(ControlMode.PercentOutput, 0);
    } else if(limitSwitch4.get() && speed < 0) { //false is closed on ON, but closed is true on NC
      motorLift.set(ControlMode.PercentOutput, 0);
    } else {
      motorLift.set(ControlMode.PercentOutput, speed);
    }
  }

  public void Intake(double angle) { //dpad input
    double Pow = .5;
    double speed;
    if(angle > 90 && angle < 270) { //dpad down
      speed = Pow; //suck in
    } else if((angle < 90 && angle > 0) || (angle > 270 && angle < 360)) { //dpad up
      speed = -Pow; //spit out
    } else {
      speed = 0;
    }
    motorIntake.set(ControlMode.PercentOutput, speed);
  }
}
