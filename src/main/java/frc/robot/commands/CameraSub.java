/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class CameraSub extends InstantCommand {
  /**
   * Add your docs here.
   */

  UsbCamera camera0;
  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;

  public CameraSub() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    camera0 = CameraServer.getInstance().startAutomaticCapture(0);
    camera1 = CameraServer.getInstance().startAutomaticCapture(1);
    camera2 = CameraServer.getInstance().startAutomaticCapture(2);
    server = CameraServer.getInstance().getServer();
  }

  // Called once when the command executes
  //boolean camEnabled = false;
  int camEnabled = 0;
  @Override
  protected void initialize() {
    SmartDashboard.putBoolean("CameraSub", true);
    SmartDashboard.putNumber("camEnabled1", camEnabled);

    switch(camEnabled) {
      case 0: //camera0 is enabled
        server.setSource(camera1);
        camEnabled = 1;
        SmartDashboard.putNumber("camEnabled2", camEnabled);
      case 1:
        server.setSource(camera2);
        camEnabled = 2;
        SmartDashboard.putNumber("camEnabled2", camEnabled);
      case 2:
        server.setSource(camera0);
        camEnabled = 0;
        SmartDashboard.putNumber("camEnabled2", camEnabled);
    }
      /*if(camEnabled){
      server.setSource(camera1);
     camEnabled =! camEnabled;
     } else /*(!m_oi.button7.get()&& !isLast) *///{
       //Netwo!kTableInstance.getDefault().getTable("").putString("CameraSelection", camera1.getName());
       /*server.setSource(camera2);
       camEnabled =! camEnabled;
     }*/
     
  }

}
