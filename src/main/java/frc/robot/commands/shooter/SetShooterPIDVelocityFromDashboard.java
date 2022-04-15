package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.WaitAndVibrateCommand;
import frc.robot.subsystems.ShooterSubsystem;

import java.math.BigDecimal;

import static frc.robot.Constants.TransferConstants.MAX_BALL_COUNT;


public class SetShooterPIDVelocityFromDashboard extends CommandBase {
    private ShooterSubsystem.ShooterLocationPreset shooterLocationPreset;

    private PIDController flywheelControllerFar;
    private PIDController flywheelControllerLow;
    private ShooterSubsystem flywheelSubsystem;

    public SetShooterPIDVelocityFromDashboard(ShooterSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;

        flywheelControllerFar = new PIDController(0.0005,0,0.000008);
        flywheelControllerLow = new PIDController(0.00025,0,0.000008);
    }
    public  SetShooterPIDVelocityFromDashboard(ShooterSubsystem flywheelSubsystem, XboxController operatorController) {
        this(flywheelSubsystem);
        flywheelSubsystem.setTargetVelocity(SmartDashboard.getNumber("Custom Velocity", 0));
        new Button(() -> flywheelSubsystem.isAtSetPoint()).whenHeld(new WaitAndVibrateCommand(operatorController, 0.05));
    }

    @Override
    public void initialize() {
        SmartDashboard.setDefaultNumber("Custom Velocity", 1200);
        SmartDashboard.setDefaultNumber("Custom Hood Angle", 0);
    }

    @Override
    public void execute() {
        double velocity = SmartDashboard.getNumber("Custom Velocity", 0);
        double hoodAngle = SmartDashboard.getNumber("Custom Hood Angle", 0);
        flywheelSubsystem.setTargetVelocity(velocity);

        double pidOutput = 0;
        if (velocity < 3500){
            pidOutput = flywheelControllerLow.calculate(flywheelSubsystem.getFlywheelRPM(), velocity);
        } else {
            pidOutput = flywheelControllerFar.calculate(flywheelSubsystem.getFlywheelRPM(), velocity);
        }

        BigDecimal KF_PERCENT_FACTOR_FLYWHEEL = new BigDecimal("0.00018082895");
        BigDecimal KF_CONSTANT = new BigDecimal("0.0159208876");

        BigDecimal feedforward = (new BigDecimal(velocity).multiply(KF_PERCENT_FACTOR_FLYWHEEL)).add(KF_CONSTANT);

        double feedForwardedPidOutput = pidOutput + feedforward.doubleValue();

        // Ensure it is never negative
        double positiveMotorOutput = (feedForwardedPidOutput <= 0) ? 0 : feedForwardedPidOutput;
        double clampedPositiveFinalMotorOutput = (positiveMotorOutput > 1) ? 1 : positiveMotorOutput;

        flywheelSubsystem.setPercentSpeed(clampedPositiveFinalMotorOutput);
        flywheelSubsystem.setHoodAngle(hoodAngle);



    }

    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.stopFlywheel();
    }
}
