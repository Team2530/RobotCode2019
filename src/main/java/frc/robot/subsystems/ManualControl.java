/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ManualControl extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public double exponentialRebind(double x){
    double controlFactor = RobotMap.exponentialControlFactor;
    if(x>0){
    return 1/(1+controlFactor)*Math.pow(controlFactor,x)-1/(1+controlFactor);
    }else{
    return -(1/(1+controlFactor)*Math.pow(controlFactor,-x)-1/(1+controlFactor));
    }
  }
}
