/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.Drive2;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {

  

  // VictorSP motor0 = new VictorSP(0);
  // VictorSP motor2 = new VictorSP(2);

  // VictorSPX motor1 = new VictorSPX(0); //id 0
  // VictorSPX motor3 = new VictorSPX(1); //id 1
  

  //below for bot, above for practice bot

  TalonSRX motor3 = new TalonSRX(3); //id 3
  TalonSRX motor4 = new TalonSRX(4); //id 4
   
  VictorSPX motor1 = new VictorSPX(1); //id 1
  VictorSPX motor2 = new VictorSPX(2); //id 2

  // VictorSP motor0;

  double y1;
  double y2;
  double x1;
  double x2;
  double z1;

  // double backleftPow;
  // double backrightPow;
  double leftPow;
  double rightPow;
  final double deadzone = 0.1;
  double powerfactor = 1;
  int driveDirection = 1;

  //gyro/navx varibles
  AHRS ahrs;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Drive2());
    
  }

  public void Drive(Joystick stick) {
    x1 = driveDirection*stick.getX();
    y1 = driveDirection*stick.getY();
    z1 = stick.getZ();
    if (x1 >= -deadzone && x1 <= deadzone) {
      x1 = 0;
    } 
    if (y1 >= -deadzone && y1 <= deadzone) {
      y1 = 0;
    } 
    if (Math.abs(z1) <= 0.3) {
      z1 = 0;
    }

    SmartDashboard.putNumber("x", x1);
    SmartDashboard.putNumber("y", y1);
    SmartDashboard.putNumber("z", z1);

    powerfactor = -stick.getRawAxis(3);
    powerfactor = 0.5 * (powerfactor + 1); //changes max power based on slider
    SmartDashboard.putNumber("powerfactor", powerfactor);

    y1 = powerfactor*(0.5 * Math.pow(y1, 3) + 0.5 * y1);
    z1 = powerfactor*(0.5 * Math.pow(z1, 3) + 0.5 * z1);

    rightPow = (y1 + z1);
    //backrightPow = (y1 - z1 + x1);
    leftPow = (y1 - z1);
    //backleftPow = (y1 + z1 - x1);
    //powerfactor = -stick.getRawAxis(4);

    

    //rightPow = powerfactor*(0.75 * Math.pow(rightPow, 3) + 0.25 * rightPow);
    //leftPow = powerfactor*(0.75 * Math.pow(leftPow, 3) + 0.25 * leftPow);

    // if(false) {
    //   motor0.set(rightPow);
    //   motor2.set(rightPow);

    //   motor1.set(ControlMode.PercentOutput, leftPow);
    //   motor3.set(ControlMode.PercentOutput, leftPow);
    // } else {
      motor3.set(ControlMode.PercentOutput, leftPow);
      motor4.set(ControlMode.PercentOutput, -rightPow);

      motor1.set(ControlMode.PercentOutput, leftPow); 
      motor2.set(ControlMode.PercentOutput, -rightPow); 
    // }
  }

  public void Drive2(Joystick stick1, Joystick stick2) {
    //x1 = stick1.getX();
    y1 = stick1.getY();
    //x2 = stick2.getX();
    y2 = stick2.getY();
    if (y2 >= -deadzone && y2 <= deadzone) {
      y2 = 0;
    } 
    if (y1 >= -deadzone && y1 <= deadzone) {
      y1 = 0;
    } 

    rightPow = (y1); 
    leftPow = (y2); //should? be tank drive

    // SmartDashboard.putNumber("RightBefore", rightPow);
    // SmartDashboard.putNumber("LeftBefore", leftPow);

    powerfactor = -stick1.getRawAxis(3);
    powerfactor = 0.5 * (powerfactor + 1); //changes max power based on slider
    SmartDashboard.putNumber("powerfactor", powerfactor);

    rightPow = powerfactor*(0.5 * Math.pow(rightPow, 3) + 0.5 * rightPow);
    leftPow = powerfactor*(0.5 * Math.pow(leftPow, 3) + 0.5 * leftPow);

    // SmartDashboard.putNumber("RightAfter", rightPow);
    // SmartDashboard.putNumber("LeftAfter", leftPow);

    // motor0.set(rightPow);
    // motor2.set(rightPow);

    // motor1.set(ControlMode.PercentOutput, leftPow);
    // motor3.set(ControlMode.PercentOutput, leftPow);

    motor3.set(ControlMode.PercentOutput, leftPow);
    motor4.set(ControlMode.PercentOutput, -rightPow);

    motor1.set(ControlMode.PercentOutput, leftPow); 
    motor2.set(ControlMode.PercentOutput, -rightPow/*.4*/); 
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

    /*rightPow = (y1 - x2 - x1);
    //backrightPow = (y1 - x2 + x1);
    leftPow = (y1 + x2 + x1);
    //backleftPow = (y1 + x2 - x1);*/

    rightPow = -(y1 + x1); 
    leftPow = (y1 - x1); //should? be tank drive
    
    // motor0.set(leftPow);
    // motor2.set(-rightPow);

    // motor1.set(ControlMode.PercentOutput, -rightPow);
    // motor3.set(ControlMode.PercentOutput, leftPow);

    //practice vs real

    motor3.set(ControlMode.PercentOutput, leftPow);
    motor4.set(ControlMode.PercentOutput, rightPow);

    motor1.set(ControlMode.PercentOutput, leftPow); 
    motor2.set(ControlMode.PercentOutput, rightPow); 
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

    rightPow = (y1); 
    leftPow = (y2); //should? be tank drive
    
    // motor0.set(leftPow);
    // motor2.set(-rightPow);

    // motor1.set(ControlMode.PercentOutput, -rightPow);
    // motor3.set(ControlMode.PercentOutput, leftPow);

    //practice vs real

    motor3.set(ControlMode.PercentOutput, -leftPow);
    motor4.set(ControlMode.PercentOutput, rightPow);

    motor1.set(ControlMode.PercentOutput, rightPow); 
    motor2.set(ControlMode.PercentOutput, -leftPow); 
  }

  public void Stop() {
    // motor0.set(0);
    // motor2.set(0);

    // motor1.set(ControlMode.PercentOutput, 0);
    // motor3.set(ControlMode.PercentOutput, 0);

    //practice vs real

    motor3.set(ControlMode.PercentOutput, 0);
    motor4.set(ControlMode.PercentOutput, 0);

    motor1.set(ControlMode.PercentOutput, 0);
    motor2.set(ControlMode.PercentOutput, 0);
  }

  public void DriveStraightForwards() {
    
    motor3.set(ControlMode.PercentOutput, 1);
    motor4.set(ControlMode.PercentOutput, -1);

    motor1.set(ControlMode.PercentOutput, 1); 
    motor2.set(ControlMode.PercentOutput, -1);
    
    System.out.println("Executed");  //comment this out later
  }

  public void SetDrivePower(double leftpower,double rightpower){

    motor3.set(ControlMode.PercentOutput, -rightpower);
    motor4.set(ControlMode.PercentOutput, leftpower);

    motor1.set(ControlMode.PercentOutput, -rightpower); 
    motor2.set(ControlMode.PercentOutput, leftpower);
  }

  public void DriveStraightBackwards() {
    
    motor3.set(ControlMode.PercentOutput, -1);
    motor4.set(ControlMode.PercentOutput, 1);

    motor1.set(ControlMode.PercentOutput, -1); 
    motor2.set(ControlMode.PercentOutput, 1);
    
    System.out.println("Executed");  //comment this out later
  }

  public void FlipDrive(){
    driveDirection *= -1;
  }

  public void initNavX() {
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      ahrs = new AHRS(SPI.Port.kMXP); 
    } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
  }

  public void resetGryo() { 
    ahrs.reset();
  }

  public double getGryo() {
   
    //double[] gyro = {ahrs.getRawGyroX(), ahrs.getRawGyroY(), ahrs.getRawGyroZ()};
    double gyro = (double) ahrs.getAngle();
    return gyro;
  }

  public void angleGyroTurn(double target,double power){
    double offset = 2;

    if(target<getGryo()-offset){
      SetDrivePower(power, -power);
    }else if(target>getGryo()+offset){
      SetDrivePower(-power, power);
    }else{

    }
  }
}
