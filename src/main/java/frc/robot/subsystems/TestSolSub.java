/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class TestSolSub extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  DoubleSolenoid sol1 = new DoubleSolenoid(0, 1);
  DoubleSolenoid sol2 = new DoubleSolenoid(2, 3);
  DoubleSolenoid endSol = new DoubleSolenoid(4, 5);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void extendSol1(){
    sol1.set(Value.kForward);
  }
  public void retractSol1(){
    sol1.set(Value.kReverse);
  }

  public boolean getSol1(){
    if(sol1.get()==Value.kForward){
      return true;//sol is extened
    }else {
      return false;
    }
  }

  public void extendSol2(){
    sol2.set(Value.kForward);
  }
  public void retractSol2(){
    sol2.set(Value.kReverse);
  }

  public boolean getSol2(){
    if(sol2.get()==Value.kForward){
      return true;//sol is extened
    }else {
      return false;
    }
  }

  public void extendEndSol(){
    endSol.set(Value.kForward);
  }
  public void retractEndSol(){
    endSol.set(Value.kReverse);
  }

  public boolean getEndSol(){
    if(endSol.get()==Value.kForward){
      return true; //sol is extened
    }else {
      return false;
    }
  }
}
