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
import frc.robot.commands.SingleJoystickDrive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

//!!!!IMPORTANT NOTE!!!!     -slot 0 = Xbox controller  -slot 1 = stick1  -slot 2 = stick2

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {

  VictorSP motor0 = new VictorSP(0); // fakeid = idk
  VictorSP motor2 = new VictorSP(2); // fakeid = idk

  VictorSPX motor1 = new VictorSPX(0); // id 0 //fakeid = 2
  VictorSPX motor3 = new VictorSPX(1); // id 1 //fakeid = 4

  // these are for joystick

  public final double deadzone = 0.1; // deadzone stuff
  public final double zDeadzone = 0.3; // deadzone for z axis turns
  double powerfactor = 1; // power multiplier
  int driveDirection = 1; // used for multidirectional drive

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new SingleJoystickDrive());
  }

  public int getDriveDirection() {
    return driveDirection;
  }

  public void setMotorPower(int port, double speed) {
    switch (port) {
    case 0:
      motor0.set(speed);
      return;
    case 1:
      motor1.set(ControlMode.PercentOutput, speed);
      return;
    case 2:
      motor2.set(speed);
      return;
    case 3:
      motor3.set(ControlMode.PercentOutput, speed);
      return;
    default:
      return;
    }

  }

  public void Stop() {
    // motor0.set(0);
    // motor2.set(0.1);

    motor1.set(ControlMode.PercentOutput, 0);
    motor3.set(ControlMode.PercentOutput, 0);

    // practice(^^) vs real(VV)

    // motor3.set(ControlMode.PercentOutput, 0);
    // motor4.set(ControlMode.PercentOutput, 0);

    // motor1.set(ControlMode.PercentOutput, 0);
    // motor2.set(ControlMode.PercentOutput, 0);
  }

  public int DriveStraight(int botDirection) {
    if (botDirection == 1) {
      motor0.set(0.1);
      motor2.set(0.1);

      motor1.set(ControlMode.PercentOutput, 0.1);
      motor3.set(ControlMode.PercentOutput, 0.1);

      // practice(^^) vs real (VV)

      // motor3.set(ControlMode.PercentOutput, 0.1);
      // motor4.set(ControlMode.PercentOutput, -0.1);

      // motor1.set(ControlMode.PercentOutput, -0.1);
      // motor2.set(ControlMode.PercentOutput, 0.1);
    }

    if (botDirection == -1) {
      motor0.set(0.1);
      motor2.set(0.1);

      motor1.set(ControlMode.PercentOutput, 0.1);
      motor3.set(ControlMode.PercentOutput, 0.1);

      // motor2.set(ControlMode.PercentOutput, 1);
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
    driveDirection *= -1;
  }
}
