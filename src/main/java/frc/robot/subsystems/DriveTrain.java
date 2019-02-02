/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.TestDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {

  VictorSP motor0 = new VictorSP(0);
  VictorSP motor2 = new VictorSP(2);

  VictorSPX motor1 = new VictorSPX(0); //id 0
  VictorSPX motor3 = new VictorSPX(1); //id 1

  //below for bot, above for practice bot

  // TalonSRX motor0 = new TalonSRX(1); //id 1
  // TalonSRX motor2 = new TalonSRX(4); //id 4
   
  // VictorSPX motor1 = new VictorSPX(2); //id 2
  // VictorSPX motor3 = new VictorSPX(3); //id 3

  double y1;
  double y2;
  double x1;
  double x2;
  double z1;

  double backLeftPow;
  double backRightPow;
  double frontLeftPow;
  double frontRightPow;
  final double deadzone = 0.2;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new TestDrive());
  }

  public void Drive(Joystick stick) {
    x1 = stick.getX();
    y1 = stick.getY();
    z1 = stick.getZ();
    if (x1 >= -deadzone && x1 <= deadzone) {
      x1 = 0;
    } else if (y1 >= -deadzone && y1 <= deadzone) {
      y1 = 0;
    } else if (z1 >= -.3 && z1 <= .3) {
      z1 = 0;
    }

    SmartDashboard.putNumber("x", x1);
    SmartDashboard.putNumber("y", y1);
    SmartDashboard.putNumber("z", z1);

    frontRightPow = (y1 - z1 - x1);
    //backRightPow = (y1 - z1 + x1);
    frontLeftPow = (y1 + z1 + x1);
    //backLeftPow = (y1 + z1 - x1);

    motor0.set(frontRightPow);
    motor2.set(frontRightPow);

    motor1.set(ControlMode.PercentOutput, frontLeftPow);
    motor3.set(ControlMode.PercentOutput, frontLeftPow);

    // motor0.set(ControlMode.PercentOutput, frontRightPow);
    // motor2.set(ControlMode.PercentOutput, frontRightPow);

    // motor1.set(ControlMode.PercentOutput, frontLeftPow); //i think in the back
    // motor3.set(ControlMode.PercentOutput, frontLeftPow); //i think in the back
  }

  public void Drive2(Joystick stick1, Joystick stick2) {
    x1 = stick1.getX();
    y1 = stick1.getY();
    x2 = stick2.getX();
    y2 = stick2.getY();

    frontRightPow = (y2); 
    frontLeftPow = (y1); //should? be tank drive

    motor0.set(frontRightPow);
    motor2.set(frontRightPow);

    motor1.set(ControlMode.PercentOutput, frontLeftPow);
    motor3.set(ControlMode.PercentOutput, frontLeftPow);

    // motor0.set(ControlMode.PercentOutput, -frontLeftPow);
    // motor2.set(ControlMode.PercentOutput, frontRightPow);

    // motor1.set(ControlMode.PercentOutput, frontRightPow); //i think in the back
    // motor3.set(ControlMode.PercentOutput, -frontLeftPow); //i think in the back
  }


  public void XboxDrive(XboxController xbox) {
    x1 = xbox.getX(Hand.kLeft);
    y1 = xbox.getY(Hand.kLeft);
    x2 = xbox.getX(Hand.kRight);
    y2 = xbox.getY(Hand.kRight);
    if (x1 >= -deadzone && x1 <= deadzone) {
      x1 = 0;
    }
    if (x2 >= -deadzone && x2 <= deadzone) {
      x2 = 0;
    }
    if (y1 >= -deadzone && y1 <= deadzone) {
      y1 = 0;
    }
    if (y2 >= -deadzone && y2 <= deadzone) {
      y2 = 0;
    }

    /*frontRightPow = (y1 - x2 - x1);
    //backRightPow = (y1 - x2 + x1);
    frontLeftPow = (y1 + x2 + x1);
    //backLeftPow = (y1 + x2 - x1);*/

    frontRightPow = (y1 + x1); 
    frontLeftPow = (y1 - x1); //should? be tank drive
    
    motor0.set(frontLeftPow);
    motor2.set(-frontRightPow);

    motor1.set(ControlMode.PercentOutput, -frontRightPow);
    motor3.set(ControlMode.PercentOutput, frontLeftPow);

    //practice vs real

    // motor0.set(ControlMode.PercentOutput, -frontLeftPow);
    // motor2.set(ControlMode.PercentOutput, frontRightPow);

    // motor1.set(ControlMode.PercentOutput, frontRightPow); //i think in the back
    // motor3.set(ControlMode.PercentOutput, -frontLeftPow); //i think in the back
  }

  public void XboxDrive2(XboxController xbox) {
    x1 = xbox.getX(Hand.kLeft);
    y1 = xbox.getY(Hand.kLeft);
    x2 = xbox.getX(Hand.kRight);
    y2 = xbox.getY(Hand.kRight);
    if (x1 >= -deadzone && x1 <= deadzone) {
      x1 = 0;
    }
    if (x2 >= -deadzone && x2 <= deadzone) {
      x2 = 0;
    }
    if (y1 >= -deadzone && y1 <= deadzone) {
      y1 = 0;
    }
    if (y2 >= -deadzone && y2 <= deadzone) {
      y2 = 0;
    }

    frontRightPow = (y1); 
    frontLeftPow = (y2); //should? be tank drive
    
    motor0.set(frontLeftPow);
    motor2.set(-frontRightPow);

    motor1.set(ControlMode.PercentOutput, -frontRightPow);
    motor3.set(ControlMode.PercentOutput, frontLeftPow);

    //practice vs real

    // motor0.set(ControlMode.PercentOutput, -frontLeftPow);
    // motor2.set(ControlMode.PercentOutput, frontRightPow);

    // motor1.set(ControlMode.PercentOutput, frontRightPow); //i think in the back
    // motor3.set(ControlMode.PercentOutput, -frontLeftPow); //i think in the back
  }

  public void Stop() {
    motor0.set(0);
    motor2.set(0);

    motor1.set(ControlMode.PercentOutput, 0);
    motor3.set(ControlMode.PercentOutput, 0);

    //practice vs real

    // motor0.set(ControlMode.PercentOutput, 0);
    // motor2.set(ControlMode.PercentOutput, 0);

    // motor1.set(ControlMode.PercentOutput, 0);
    // motor3.set(ControlMode.PercentOutput, 0);
  }
}
