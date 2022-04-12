package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.WaitAndVibrateCommand;
import frc.robot.hardware.Limelight;
import frc.robot.helper.shooter.ShooterState;
import frc.robot.subsystems.ShooterSubsystem;

import java.math.BigDecimal;

public class SetShooterPIDFromInterpolation extends CommandBase {
    private PIDController flywheelControllerFar;
    private PIDController flywheelControllerLow;

    private ShooterSubsystem shooterSubsystem;

    public SetShooterPIDFromInterpolation(ShooterSubsystem flywheelSubsystem) {
        this.shooterSubsystem = flywheelSubsystem;

        flywheelControllerFar = new PIDController(0.0005,0,0.000008);
        flywheelControllerLow = new PIDController(0.00025,0,0.000008);
    }

    public SetShooterPIDFromInterpolation(ShooterSubsystem flywheelSubsystem, XboxController operatorController) {
        this(flywheelSubsystem);
//        new Button(() -> flywheelSubsystem.isAtSetPoint(shooterStateSupplier.get().rpmVelocity)).whenPressed(new WaitAndVibrateCommand(operatorController, 0.5, 0.1));
    }

    @Override
    public void initialize() {
        System.out.println("Velocity PID Ramping Up");
        Limelight.enable();
    }

    @Override
    public void execute() {
        double pidOutput = 0;

        double currentDistance = Limelight.getRawDistanceToTarget();
        double targetVelocity = shooterSubsystem.getFlywheelRPMFromInterpolator(currentDistance);
        shooterSubsystem.setTargetVelocity(targetVelocity);
        double targetHoodAngle = shooterSubsystem.getHoodAngleFromInterpolator(currentDistance);

        SmartDashboard.putNumber("Interpolation Target Velocity", targetVelocity);
        SmartDashboard.putNumber("Interpolation Target Hood Angle", targetHoodAngle);

        if (targetVelocity < 3500){
            pidOutput = flywheelControllerLow.calculate(shooterSubsystem.getFlywheelRPM(), targetVelocity);
        } else {
            pidOutput = flywheelControllerFar.calculate(shooterSubsystem.getFlywheelRPM(), targetVelocity);
        }

        BigDecimal KF_PERCENT_FACTOR_FLYWHEEL = new BigDecimal("0.00018082895");
        BigDecimal KF_CONSTANT = new BigDecimal("0.0159208876");

        BigDecimal feedforward = (new BigDecimal(targetVelocity).multiply(KF_PERCENT_FACTOR_FLYWHEEL)).add(KF_CONSTANT);

        double feedForwardedPidOutput = pidOutput + feedforward.doubleValue();

        // Ensure it is never negative
        double positiveMotorOutput = (feedForwardedPidOutput <= 0) ? 0 : feedForwardedPidOutput;
        double clampedPositiveFinalMotorOutput = (positiveMotorOutput > 1) ? 1 : positiveMotorOutput;

        shooterSubsystem.setPercentSpeed(clampedPositiveFinalMotorOutput);
        shooterSubsystem.setHoodAngle(targetHoodAngle);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFlywheel();
        Limelight.disable();
    }

}
