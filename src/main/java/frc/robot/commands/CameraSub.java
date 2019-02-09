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

/**
 * Add your docs here.
 */
public class CameraSub extends InstantCommand {
  /**
   * Add your docs here.
   */
  public CameraSub() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    camera2 = CameraServer.getInstance().startAutomaticCapture(1);
    server = CameraServer.getInstance().getServer();
  }

  // Called once when the command executes
  boolean camEnabled = false;
  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;
  @Override
  protected void initialize() {
      if(camEnabled){
      server.setSource(camera1);
     camEnabled =! camEnabled;
     } else /*(!m_oi.button7.get()&& !isLast) */{
       //Netwo!kTableInstance.getDefault().getTable("").putString("CameraSelection", camera1.getName());
       server.setSource(camera2);
       camEnabled =! camEnabled;
     }
     
  }

}
