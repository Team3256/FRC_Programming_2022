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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.shooter.SetShooterPIDVelocityFromDashboard;
import frc.robot.commands.transfer.TransferShootForward;

import frc.robot.hardware.Limelight;
import frc.robot.helper.logging.RobotLogger;
import io.github.oblarg.oblog.Logger;

public class Robot extends TimedRobot {
  private static final RobotLogger logger = new RobotLogger(Robot.class.getCanonicalName());

  private Command autonomousCommand;
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    RobotLogger.init();
    robotContainer = new RobotContainer();
    SmartDashboard.putData("Auto Chooser", robotContainer.getCommandChooser());

    Logger.configureLoggingAndConfig(robotContainer, false);
  }

  @Override
  public void robotPeriodic() {
    Logger.updateEntries();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    Limelight.disable();
    logger.info("Robot Disabled");
  }

  @Override
  public void autonomousInit() {
    Limelight.enable();
    logger.info("Auto Enabled");
    robotContainer.resetPose();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }  else {
      logger.severe("NOT RUNNING AUTO COMMAND: AUTO COMMAND IS NULL");
    }
  }

  @Override
  public void teleopInit() {
    Limelight.enable();
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

    robotContainer.zeroSubsystems();
  }

  @Override
  public void testPeriodic() {}
}
