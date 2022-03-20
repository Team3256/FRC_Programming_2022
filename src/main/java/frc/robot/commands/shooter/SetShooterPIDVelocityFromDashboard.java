package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

import java.math.BigDecimal;
import java.util.function.DoubleSupplier;


public class SetShooterPIDVelocityFromDashboard extends CommandBase {
    private FlywheelSubsystem.ShooterLocationPreset shooterLocationPreset;

    private PIDController flywheelControllerFar;
    private PIDController flywheelControllerLow;
    private DoubleSupplier flywheelRPMSupplier;

    private FlywheelSubsystem flywheelSubsystem;
    public SetShooterPIDVelocityFromDashboard(FlywheelSubsystem flywheelSubsystem, DoubleSupplier flywheelRPMSupplier) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.flywheelRPMSupplier = flywheelRPMSupplier;

        flywheelControllerFar = new PIDController(0.0005,0,0.000008);
        flywheelControllerLow = new PIDController(0.00025,0,0.000008);

        SmartDashboard.putData("Flywheel LOW", flywheelControllerLow);
    }

    @Override
    public void initialize() {

        SmartDashboard.setDefaultNumber("Custom Velocity", 1000);
        SmartDashboard.setDefaultNumber("Custom Hood Angle", 0);
    }

    @Override
    public void execute() {


        double velocity = SmartDashboard.getNumber("Custom Velocity", 0);
        double hoodAngle = SmartDashboard.getNumber("Custom Hood Angle", 0);

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

        SmartDashboard.putNumber("Flywheel Output",clampedPositiveFinalMotorOutput);

    }

    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.stopFlywheel();
    }
}
