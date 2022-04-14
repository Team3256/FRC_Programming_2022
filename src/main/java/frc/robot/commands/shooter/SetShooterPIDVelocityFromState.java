package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.WaitAndVibrateCommand;
import frc.robot.helper.shooter.ShooterState;
import frc.robot.subsystems.ShooterSubsystem;

import java.math.BigDecimal;
import java.util.function.Supplier;


public class SetShooterPIDVelocityFromState extends CommandBase {
    private PIDController flywheelControllerFar;
    private PIDController flywheelControllerLow;

    private ShooterSubsystem flywheelSubsystem;
    private Supplier<ShooterState> shooterStateSupplier;

    public SetShooterPIDVelocityFromState(ShooterSubsystem flywheelSubsystem, Supplier<ShooterState> shooterStateSupplier) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.shooterStateSupplier = shooterStateSupplier;

        flywheelControllerFar = new PIDController(0.0005,0,0.000008);
        flywheelControllerLow = new PIDController(0.00025,0,0.000008);
       }

   public SetShooterPIDVelocityFromState(ShooterSubsystem flywheelSubsystem, Supplier<ShooterState> shooterStateSupplier, XboxController operatorController) {
        this(flywheelSubsystem, shooterStateSupplier);
        new Button(() -> flywheelSubsystem.isAtSetPoint()).whenPressed(new WaitAndVibrateCommand(operatorController, 0.5, 0.1));
   }

    @Override
    public void initialize() {

        System.out.println("Velocity PID Ramping Up");
    }

    @Override
    public void execute() {

        double pidOutput;

        if (shooterStateSupplier.get().rpmVelocity < 3500){
            pidOutput = flywheelControllerLow.calculate(flywheelSubsystem.getFlywheelRPM(), shooterStateSupplier.get().rpmVelocity);
        } else {
            pidOutput = flywheelControllerFar.calculate(flywheelSubsystem.getFlywheelRPM(), shooterStateSupplier.get().rpmVelocity);
        }

        BigDecimal KF_PERCENT_FACTOR_FLYWHEEL = new BigDecimal("0.00018082895");
        BigDecimal KF_CONSTANT = new BigDecimal("0.0159208876");

        BigDecimal feedforward = (new BigDecimal(shooterStateSupplier.get().rpmVelocity).multiply(KF_PERCENT_FACTOR_FLYWHEEL)).add(KF_CONSTANT);

        double feedForwardedPidOutput = pidOutput + feedforward.doubleValue();

        // Ensure it is never negative
        double positiveMotorOutput = (feedForwardedPidOutput <= 0) ? 0 : feedForwardedPidOutput;
        double clampedPositiveFinalMotorOutput = (positiveMotorOutput > 1) ? 1 : positiveMotorOutput;

        flywheelSubsystem.setPercentSpeed(clampedPositiveFinalMotorOutput);
        flywheelSubsystem.setHoodAngle(shooterStateSupplier.get().hoodAngle);
    }

    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.stopFlywheel();
    }

}
