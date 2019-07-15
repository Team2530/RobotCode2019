/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.RobotMap;
import frc.robot.commands.SingleJoystickDrive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

//!!!!IMPORTANT NOTE!!!!     -slot 0 = Xbox controller  -slot 1 = stick1  -slot 2 = stick2

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new SingleJoystickDrive());
  }

  public void setMotorPower(int port, double speed) {
    switch (port) {
    case 0:
      RobotMap.motor0.set(speed);
      return;
    case 1:
      RobotMap.motor1.set(ControlMode.PercentOutput, speed);
      return;
    case 2:
      RobotMap.motor2.set(speed);
      return;
    case 3:
      RobotMap.motor3.set(ControlMode.PercentOutput, speed);
      return;
    default:
      return;
    }

  }
  public void Stop() {
    for (int i = 0; i < 3; i++) {
      setMotorPower(i, 0);
    }
  }

  // ! do not use
  //public int DriveStraight(int power) {
    if (botDirection == 1) {
      
    }

    if (botDirection == -1) {
      RobotMap.motor0.set(0.1);
      RobotMap.motor2.set(0.1);

      RobotMap.motor1.set(ControlMode.PercentOutput, 0.1);
      RobotMap.motor3.set(ControlMode.PercentOutput, 0.1);

    }
    System.out.println("Executed"); // comment this out later
    return botDirection;
  }

  public void SetDrivePower(double leftpower, double rightpower) {

    // motor3.set(ControlMode.PercentOutput, -rightpower);
    // motor4.set(ControlMode.PercentOutput, leftpower);

    // motor1.set(ControlMode.PercentOutput, -rightpower);
    // motor2.set(ControlMode.PercentOutput, leftpower);

    // Real(^^)

  }

  public void FlipDrive() {
    RobotMap.driveDirection *= -1;
  }
}
