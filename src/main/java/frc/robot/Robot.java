/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.commands.hanger.HangerZeroRetract;
import frc.robot.commands.shooter.ZeroHoodMotorCommand;
import frc.robot.helper.logging.RobotLogger;

public class Robot extends TimedRobot {
  private static final RobotLogger logger = new RobotLogger(Robot.class.getCanonicalName());

  private Command autonomousCommand;
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    RobotLogger.init();
    robotContainer = new RobotContainer();
    LiveWindow.disableAllTelemetry();
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
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    logger.info("Auto Enabled");
    robotContainer.resetPose();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule(false);
    }
  }

  @Override
  public void autonomousPeriodic() { }

  @Override
  public void teleopInit() {
    logger.info("TeleOp Enabled");
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    LiveWindow.setEnabled(false);
    logger.info("Test Enabled");
    CommandScheduler.getInstance().cancelAll();

    CommandScheduler.getInstance().schedule(new ZeroHoodMotorCommand(robotContainer.flywheelSubsystem));
    CommandScheduler.getInstance().schedule(new HangerZeroRetract(robotContainer.hangerSubsystem));
  }

  @Override
  public void testPeriodic() {


  }
}