/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Turret extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  VictorSPX motor0 = new VictorSPX(2);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void Rotate(double speed) {
    motor0.set(ControlMode.PercentOutput, speed);
  }

  public void Stop() {
    motor0.set(ControlMode.PercentOutput, 0);
  }
}
