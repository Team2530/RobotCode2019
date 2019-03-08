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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.RotateTurret;
import frc.robot.commands.RotateTurretDegrees;
import frc.robot.commands.TurrentInit;

/**
 * Add your docs here.
 */
public class Turret extends Subsystem {
  boolean executed;
  boolean limit2active;
  boolean limit1active;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //VictorSPX motor0 = new VictorSPX(2);
  DigitalInput limitSwitch1 = new DigitalInput(1);
  DigitalInput limitSwitch2 = new DigitalInput(2);
  Encoder encoder = new Encoder(3, 4);
  VictorSPX motor0 = new VictorSPX(5);  //id 5  //3 ->5

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new RotateTurretDegrees(0));
    setDefaultCommand(new RotateTurret(0));
    encoder.setDistancePerPulse((double) 1);
  }
  


  public Boolean InitExecuted(boolean executed){
    return executed;
  }

  public DigitalInput getlimitSwitch1(){
    return limitSwitch1;
  }
  public DigitalInput getlimitSwitch2(){
    return limitSwitch2;
  }
  public boolean getLimit1Value(){
    return limitSwitch1.get();
    
  }
  public boolean getLimit2Value(){
    return limitSwitch2.get();
      

    }
  

  public double getEncoderValue(){
    return encoder.getDistance();
  }
  
  public void Rotate(double speed) {
    SmartDashboard.putBoolean("LimitSwitch1", limitSwitch1.get());
    SmartDashboard.putBoolean("LimitSwitch2", limitSwitch2.get());

    if(limitSwitch1.get() && speed > 0) { //false is closed on NO, but closed is true on NC
      motor0.set(ControlMode.PercentOutput, 0);
      encoder.reset();
    } else if(limitSwitch2.get() && speed < 0) { //false is closed on ON, but closed is true on NC
      motor0.set(ControlMode.PercentOutput, 0);
    } else {
      motor0.set(ControlMode.PercentOutput, speed);
    }
      //when initialized:
     /*if((executed = true) && (speed < 0)) { //tests if you want to set it to 0degrees and it rotates counterclockwise
      motor0.set(ControlMode.PercentOutput, speed);
      while(speed != 0){
        [command that checks limitswitch]
      }
     } else {
      motor0.set(ControlMode.PercentOutput, speed);
    } */
  }

  public void Stop() {
    motor0.set(ControlMode.PercentOutput, 0);
  }
}
