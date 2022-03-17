package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.ALL_SHOOTER_PRESETS;


public class ShooterPIDCommand extends CommandBase {

    private PIDController flywheelController;

    private FlywheelSubsystem flywheelSubsystem;
    private HoodSubsystem hoodSubsystem;
    private DoubleSupplier flywheelRPM;

    public ShooterPIDCommand(FlywheelSubsystem flywheelSubsystem, HoodSubsystem hoodSubsystem, DoubleSupplier flywheelRPM) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.flywheelRPM = flywheelRPM;
        addRequirements(flywheelSubsystem, hoodSubsystem);
        flywheelController = new PIDController(0,0,0);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
            flywheelController.setSetpoint(flywheelRPM.getAsDouble());

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
