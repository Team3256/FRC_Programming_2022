/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helper.LED2.LEDController;
import frc.robot.helper.logging.RobotLogger;

public class Robot extends TimedRobot {
  private static final RobotLogger logger = new RobotLogger(Robot.class.getCanonicalName());

  private Command autonomousCommand;
  private RobotContainer robotContainer;

  //LEDSubsystem ledSubsystem = new LEDSubsystem(100, 0);
  LEDController ledController = new LEDController();

  @Override
  public void robotInit() {
    RobotLogger.init();
    robotContainer = new RobotContainer();
    SmartDashboard.putData(robotContainer.getCommandChooser());
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
    //ledSubsystem.rainbow();
    ledController.periodic();
  }

  @Override
  public void disabledInit() {
    logger.info("Robot Disabled");
  }

  @Override
  public void autonomousInit() {
    logger.info("Auto Enabled");
    robotContainer.resetPose();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }



  @Override
  public void teleopInit() {
    logger.info("TeleOp Enabled");
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }


  @Override
  public void testInit() {
    logger.info("Test Enabled");
    CommandScheduler.getInstance().cancelAll();
    //ledSubsystem.setAll(0,0,255);
  }
}