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

  VictorSPX motorLift1 = new VictorSPX(6);
  VictorSPX motorLift2 = new VictorSPX(7); //two motors for lifting
  VictorSPX motorIntake = new VictorSPX(8);
  DigitalInput limitSwitchTop = new DigitalInput(5);
  DigitalInput limitSwitchBottom = new DigitalInput(6);
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void UpAndDown(double speed) { //xbox joystick input
    SmartDashboard.putBoolean("LimitSwitchTop", limitSwitchTop.get());
    SmartDashboard.putBoolean("LimitSwitchBottom", limitSwitchBottom.get());
    if(limitSwitchBottom.get() && speed > 0) { //false is closed on NO, but closed is true on NC
      motorLift1.set(ControlMode.PercentOutput, 0);
      motorLift2.set(ControlMode.PercentOutput, 0);
    } else if(limitSwitchTop.get() && speed < 0) { //false is closed on ON, but closed is true on NC
      motorLift1.set(ControlMode.PercentOutput, 0);
      motorLift2.set(ControlMode.PercentOutput, 0);
    } else {
      motorLift1.set(ControlMode.PercentOutput, speed);
      motorLift2.set(ControlMode.PercentOutput, speed);
    }
  }

  public void Intake(double angle) { //dpad input
    double Pow = .5;
    double speed;
    if(angle > 90 && angle < 270) { //dpad down
      speed = Pow; //succ in
    } else if((angle < 90 && angle > 0) || (angle > 270 && angle < 360)) { //dpad up
      speed = -Pow; //spit out
    } else {
      speed = 0;
    }
    motorIntake.set(ControlMode.PercentOutput, speed);
  }

  public boolean getLimit3Value(){
    return limitSwitchTop.get();
  }
  public boolean getLimit4Value(){
    return limitSwitchBottom.get();
  }
}
