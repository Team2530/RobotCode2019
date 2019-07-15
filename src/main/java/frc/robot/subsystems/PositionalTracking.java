/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;


import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class PositionalTracking extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  

  public AHRS ahrs = new AHRS(SPI.Port.kMXP); 
  Float[] rawgyro = new Float[3];
  Float[] rawacclerometer= new Float[3];
  Float rawcompass;
  Float fusedheading;

  boolean motionDetected;
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void initAHRS(){
    ahrs.reset();
    motionDetected = ahrs.isMoving();
    rawacclerometer[0] = ahrs.getRawAccelX();
    rawacclerometer[1] = ahrs.getRawAccelY();
    rawacclerometer[2] = ahrs.getRawAccelZ();
    rawgyro[0] = ahrs.getRawGyroX();
    rawgyro[1] = ahrs.getRawGyroY();
    rawgyro[2] = ahrs.getRawGyroZ();
    fusedheading = ahrs.getFusedHeading();
  }
  //TODO 
  public void getEncoders(){

  }
}
