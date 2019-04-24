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
  DigitalInput limitSwitchTop = new DigitalInput(0);
  // DigitalInput limitSwitchBottom = new DigitalInput(5);
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void UpAndDown(double speed) { //xbox joystick input
    SmartDashboard.putBoolean("LimitSwitchTop", limitSwitchTop.get());
    // SmartDashboard.putBoolean("LimitSwitchBottom", limitSwitchBottom.get());
    /*if(limitSwitchBottom.get() && speed > 0) { //false is closed on NO, but closed is true on NC
      motorLift1.set(ControlMode.PercentOutput, 0);
      motorLift2.set(ControlMode.PercentOutput, 0);
     } else*/ if(limitSwitchTop.get() && speed < 0) { //false is closed on ON, but closed is true on NC
      motorLift1.set(ControlMode.PercentOutput, 0);
      motorLift2.set(ControlMode.PercentOutput, 0);
    } else {
      motorLift1.set(ControlMode.PercentOutput, speed/2.5);
      motorLift2.set(ControlMode.PercentOutput, speed/2.5);
    }
  }


  public void Intake(double angle) { //dpad input
    double Pow = 1;
    double speed;
    SmartDashboard.putNumber("dpad", angle);
    if(angle > 90 && angle < 270) { //dpad down
      SmartDashboard.putString("dpad direction", "down");
      speed = -Pow; //spit out
    } else if((angle < 90 && angle >= 0) || (angle > 270 && angle < 360)) { //dpad up
      SmartDashboard.putString("dpad direction", "up");
      speed = Pow; //succ in
    } else {
      SmartDashboard.putString("dpad direction", "none");
      speed = 0;
    }
    SmartDashboard.putNumber("intake speed", speed);
    motorIntake.set(ControlMode.PercentOutput, speed);
  }

  public boolean getLimit3Value(){
    return limitSwitchTop.get();
  }
  // public boolean getLimit4Value(){
  //   return limitSwitchBottom.get();
  // }
}
