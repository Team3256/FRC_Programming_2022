package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FlywheelSubsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.ALL_SHOOTER_PRESETS;

//shooter pid from preset
public class ShooterPIDCommand extends CommandBase {
    private FlywheelSubsystem.ShooterLocationPreset shooterLocationPreset;

    private PIDController flywheelController;

    private FlywheelSubsystem flywheelSubsystem;
    public ShooterPIDCommand(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;

        flywheelController = new PIDController(0,0,0);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (flywheelSubsystem.getShooterLocationPreset() != this.shooterLocationPreset){
            this.shooterLocationPreset = flywheelSubsystem.getShooterLocationPreset();

            flywheelSubsystem.setHoodAngle(flywheelSubsystem.getFlywheelShooterStateFromPreset().hoodAngle);
            flywheelController.setSetpoint(flywheelSubsystem.getFlywheelShooterStateFromPreset().rpmVelocity);
        }

        double pidOutput = flywheelController.calculate(flywheelSubsystem.getFlywheelRPM());


        double KF_FLYWHEEL = 0;

        double feedforward = flywheelSubsystem.getFlywheelRPM() * KF_FLYWHEEL;

        double feedForwardedPidOutput = pidOutput + feedforward;

        // Ensure it is never negative
        double positiveFinalMotorOutput = (feedForwardedPidOutput <= 0) ? 0 : feedForwardedPidOutput;

        flywheelSubsystem.setPercentSpeed(positiveFinalMotorOutput);

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
