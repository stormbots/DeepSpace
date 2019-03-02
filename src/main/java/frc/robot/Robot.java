/*---3692+-------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.stormbots.devices.pixy2.Pixy2;


import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.CameraServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI.Port;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.WristHoming;
import frc.robot.subsystems.ArmElevator;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;

import frc.robot.subsystems.LimeLight;

import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PassThrough;
import frc.robot.subsystems.Pogos;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static boolean isCompbot = true;
  public static Compressor compressor = new Compressor();
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();

  public LimeLight limelight = new LimeLight();

  public static ArmElevator armLift = new ArmElevator();
  public static Hand hand = new Hand();
  //public static ShuffleboardTab driveTab = Shuffleboard.getTab("Match Dashboard");
  public static Pogos pogos = new Pogos();

  public static Pixy2 pixy = new Pixy2(Port.kOnboardCS0);

  public static Chassis chassis = new Chassis(); //new ChassisTalonSRX();
  public static PowerDistributionPanel pdp = new PowerDistributionPanel();
  
  public static Intake intake = new Intake();
  public static PassThrough passThrough = new PassThrough();
  public static OI m_oi = new OI();

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  Command wristHoming = new WristHoming();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   * 
   * Different than a constructor as this is run when SmartDashboard and network functions
   * are operational.
   */
  @Override
  public void robotInit() {
    //Deal with robot-specific differences
    // Don't change this code: Instead, change the value in SmartDashboard Preferences to "false".
    // This ensures that in case of any error, the code defaults to comp bot for competition safety
    if(!Preferences.getInstance().containsKey("compbot")){
      Preferences.getInstance().putBoolean("compbot", true); 
    }
    isCompbot = Preferences.getInstance().getBoolean("compbot", true);
    SmartDashboard.putBoolean("Combot?", isCompbot);
    

    armLift.robotInit();
    chassis.robotInit();
    hand.robotInit();
    intake.robotInit();
    passThrough.robotInit();
    pogos.robotInit();

    compressor.clearAllPCMStickyFaults();

    // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    // SmartDashboard.putData("Auto mode", m_chooser);
    CameraServer.getInstance().startAutomaticCapture(0);
    CameraServer.getInstance().startAutomaticCapture(1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   ** this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    SmartDashboard.putData(wristHoming);

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
    m_autonomousCommand = m_chooser.getSelected();
    if(!armLift.wrist.isHomed()){
      wristHoming.start();
    }

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
    // Don't do anything special for autonomousPeriodic: Just run teleopPeriodic instead
    teleopPeriodic();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if(!armLift.wrist.isHomed()){
      wristHoming.start();
    }
  }

  
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    limelight.update();
    armLift.update();
    hand.update();
    intake.update();
    passThrough.update();

    pogos.update();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

}
