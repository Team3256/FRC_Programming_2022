/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.commands.hanger.HangerZeroRetract;
import frc.robot.commands.shooter.ZeroHoodMotorCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.subsystems.ColorsensorTestSubsystem;

import java.awt.*;
import java.util.logging.Logger;

public class Robot extends TimedRobot {
  private static final RobotLogger logger = new RobotLogger(Robot.class.getCanonicalName());

  private Command autonomousCommand;
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    RobotLogger.init();
    robotContainer = new RobotContainer();
    robotContainer.sendCommandChoosers();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    LiveWindow.setEnabled(false);
    logger.info("Test Enabled");
    CommandScheduler.getInstance().cancelAll();

    CommandScheduler.getInstance().schedule(new ZeroHoodMotorCommand(robotContainer.shooterSubsystem));
    CommandScheduler.getInstance().schedule(new HangerZeroRetract(robotContainer.hangerSubsystem));
  }

  @Override
  public void testPeriodic() {}
}