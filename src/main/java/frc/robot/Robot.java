/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.DisplayManager;
import frc.robot.subsystem.PortMan;
import frc.robot.subsystem.SubsystemFactory;
import frc.robot.subsystem.controlpanel.ControlPanel;
import frc.robot.util.OzoneLogger;

//2910 auton stuff
import frc.common.auton.AutonomousSelector;
import frc.common.auton.AutonomousTrajectories;
import frc.common.commands.FollowTrajectoryCommand;
import frc.robot.subsystem.swerve.DrivetrainSubsystem2910;


import frc.robot.subsystem.SBInterface;
import frc.robot.subsystem.controlpanel.ControlPanelSBTab;

import java.time.Instant;
import java.time.Duration;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  static Logger logger = Logger.getLogger(Robot.class.getName());
  ControlPanel controlPanel;
  private static SubsystemFactory subsystemFactory;

  private static Instant initTime;
  private static Instant currentTime;

  private DisplayManager dManager;
  private ShuffleboardTab tab;

  private AutonomousTrajectories autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem2910.CONSTRAINTS);
  private AutonomousSelector autonomousSelector = new AutonomousSelector(autonomousTrajectories);

  private Command autonomousCommand = null;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    System.out.println("+++++++++++++++++++++++++++++++++++++ I Am here ++++++++++++++++++++++");
    resetTime();
    initTime = Instant.now();

    subsystemFactory = SubsystemFactory.getInstance();

    tab = Shuffleboard.getTab("Auton");

    OzoneLogger.getInstance().init(Level.ALL);
    logger.log(Level.INFO, "robot init");

    dManager = new DisplayManager();

    try {
      subsystemFactory.init(dManager, PortMan.getInstance());

    } catch (Exception e) {
      StringWriter writer = new StringWriter();
      PrintWriter pw  = new PrintWriter(writer);
      e.printStackTrace(pw);
      logger.severe(writer.toString());
    }

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
      CommandScheduler.getInstance().run();
      dManager.update();
      // need to double check if default Drive command is being called too.
      // this looks realy weird.
      currentTime = Instant.now();
      double elapsedTime = Duration.between(initTime, currentTime).toMillis();
      elapsedTime /= 1000;
      SubsystemFactory.getInstance().getDriveTrain().updateKinematics(elapsedTime);
       
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    SubsystemFactory.getInstance().getDriveTrain().stopSnap();
    resetTime();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    autonomousCommand = autonomousSelector.getCommand();
    autonomousCommand.schedule();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    //test.execute();
  }
  @Override
  public void teleopInit() {
    SubsystemFactory.getInstance().getDriveTrain().stopSnap();
    resetTime();
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
  public static void resetTime() {
    initTime = Instant.now();
    currentTime = Instant.now();

  }
}
