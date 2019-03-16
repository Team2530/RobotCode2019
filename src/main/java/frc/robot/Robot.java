/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;

// import edu.wpi.cscore.UsbCamera;
// import edu.wpi.cscore.VideoSink;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.Drive2;
import frc.robot.commands.TestDriveXbox;
import frc.robot.commands.TestDriveXbox2;
//import frc.robot.subsystems.ExampleSubsystem;;
import frc.robot.subsystems.*;
// import edu.wpi.cscore.CvSink;
// import edu.wpi.cscore.CvSource;
// import org.opencv.core.Mat;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static DriveTrain driveTrain = new DriveTrain();
  public static Turret turret = new Turret(); 
  public static OI m_oi;
  public static TestSolSub sol = new TestSolSub();
  public static Camera camera = new Camera();
  public static IntakeSub intake = new IntakeSub();
  //AHRS ahrs;
  boolean motionDetected;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  //NetworkTableEntry testEntry;
  NetworkTable table;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  
  
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    table = NetworkTableInstance.getDefault().getTable("datatable");
    driveTrain.initNavX();
    //testEntry = table.getEntry("time2");
    //
    // try {
    //   /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
    //   /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
    //   /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
    //   ahrs = new AHRS(SPI.Port.kMXP); 
    // } catch (RuntimeException ex ) {
    //   DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    // }
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = new Drive2();//m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  AnalogInput exampleAnalog = new AnalogInput(0);
  DigitalInput magnet = new DigitalInput(9);
  
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SmartDashboard.putNumber("Pi to RoboRio N", -1);

    

    AnalogInput.setGlobalSampleRate(62500);
    Command drive = new Drive2();
    // ahrs.resetDisplacement();
    
    drive.start();
  }

  /**
   * This function is called periodically during operator control.
   */
// double acclx;
// double accly;
// double acclz;
// double gyrox;
// double gyroy;
// double gyroz;
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    // SmartDashboard.putNumber("Analog", exampleAnalog.getValue());
    // SmartDashboard.putNumber("AnalogV", exampleAnalog.getVoltage());
    //SmartDashboard.putNumber("AnalogAverage", exampleAnalog.getAverageValue());
    //SmartDashboard.putNumber("AnalogAverageV", exampleAnalog.getAverageVoltage());
    // SmartDashboard.putNumber("Pi to RoboRio N",table.getEntry("test").getDouble(-1));
    
    SmartDashboard.putBoolean("Line Found",Boolean.parseBoolean(table.getEntry("lineFound").getString("false")));
    SmartDashboard.putNumber("r1",(table.getEntry("r1").getDouble(-1)));
    SmartDashboard.putNumber("t1",(table.getEntry("t1").getDouble(-1)));
    
    // SmartDashboard.putBoolean("Magnet", magnet.get());

    /* SmartDashboard.putNumber("r1", table.getEntry("r1").getDouble(-1));
    SmartDashboard.putNumber("t1", table.getEntry("t1").getDouble(600000)); */
    //SmartDashboard.putNumber("leftpow",table.getEntry("leftpow").getDouble(15));
    //SmartDashboard.putNumber("rightpow",table.getEntry("rightpow").getDouble(15));
    
    // motionDetected = ahrs.isMoving();
    // acclx = ahrs.getRawAccelX();
    // accly = ahrs.getRawAccelY();
    // acclz = ahrs.getRawAccelZ();
    // gyrox = ahrs.getRawGyroX();
    // gyroy = ahrs.getRawGyroY();
    // gyroz = ahrs.getRawGyroZ();

    

    // SmartDashboard.putBoolean("MotionDetected", motionDetected);
    // SmartDashboard.putNumber("Acclx", acclx);
    // SmartDashboard.putNumber("Accly", accly);
    // SmartDashboard.putNumber("Acclz", acclz);
    SmartDashboard.putNumber("gyro", driveTrain.getGryo());
    table.getEntry("gyro").setDouble(driveTrain.getGryo());
    //SmartDashboard.putNumber("gyroy", driveTrain.getGryo());
   // SmartDashboard.putNumber("gyroz", driveTrain.getGryo());
    // SmartDashboard.putNumber("displacementx", ahrs.getDisplacementX());
    // SmartDashboard.putNumber("displacementy", ahrs.getDisplacementY());
    // SmartDashboard.putNumber("displacementz", ahrs.getDisplacementZ());

    //m_oi.button7.whenReleased(new Co);
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }


  
}
