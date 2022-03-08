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

import frc.robot.commands.drivetrain.ResetPoseCommand;
import frc.robot.helper.logging.RobotLogger;

import static frc.robot.Constants.DEBUG;
import static frc.robot.Constants.SubsystemEnableFlags.DRIVETRAIN;

import frc.robot.helper.logging.RobotLogger;


public class Robot extends TimedRobot {
  private static final RobotLogger logger = new RobotLogger(Robot.class.getCanonicalName());

  private Command autonomousCommand;
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    RobotLogger.init();
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    logger.info("Robot Disabled");

    if(DEBUG) robotContainer.drivetrainSubsystem.sendTrajectoryToDashboard(robotContainer.field);

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    logger.info("Auto Enabled");
    
    if (DRIVETRAIN) {
      new ResetPoseCommand(robotContainer.drivetrainSubsystem).schedule();
      if (DEBUG) robotContainer.drivetrainSubsystem.sendTrajectoryToDashboard(robotContainer.field);
      autonomousCommand = robotContainer.getAutonomousCommand();
    }


    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    if(DRIVETRAIN && DEBUG) robotContainer.drivetrainSubsystem.autoOutputToDashboard(robotContainer.field);
  }

  @Override
  public void teleopInit() {
    logger.info("TeleOp Enabled");
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    if (DRIVETRAIN && DEBUG) robotContainer.drivetrainSubsystem.autoOutputToDashboard(robotContainer.field);
  }

  @Override
  public void testInit() {
    logger.info("Test Enabled");
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}