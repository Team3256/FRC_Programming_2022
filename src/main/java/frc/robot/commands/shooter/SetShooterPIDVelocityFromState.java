package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.WaitAndVibrateCommand;
import frc.robot.helper.shooter.ShooterState;
import frc.robot.subsystems.FlywheelSubsystem;

import java.math.BigDecimal;
import java.util.function.DoubleSupplier;


public class SetShooterPIDVelocityFromState extends CommandBase {
    private FlywheelSubsystem.ShooterLocationPreset shooterLocationPreset;

    private PIDController flywheelControllerFar;
    private PIDController flywheelControllerLow;
    private DoubleSupplier flywheelRPMSupplier;

    private FlywheelSubsystem flywheelSubsystem;
    private ShooterState shooterState;

    public SetShooterPIDVelocityFromState(FlywheelSubsystem flywheelSubsystem, ShooterState shooterState) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.shooterState = shooterState;
        this.flywheelRPMSupplier = flywheelRPMSupplier;

        flywheelControllerFar = new PIDController(0.0005,0,0.000008);
        flywheelControllerLow = new PIDController(0.00025,0,0.000008);
       }

   public  SetShooterPIDVelocityFromState(FlywheelSubsystem flywheelSubsystem, ShooterState shooterState, XboxController operatorController) {
        this(flywheelSubsystem, shooterState);
        new Button(() -> flywheelSubsystem.isAtSetPoint(shooterState.rpmVelocity)).whenPressed(new WaitAndVibrateCommand(operatorController, 0.5, 0.1));
   }

    @Override
    public void initialize() {

        System.out.println("Velocity PID Ramping Up");
    }

    @Override
    public void execute() {


        double pidOutput = 0;
        if (shooterState.rpmVelocity < 3500){
            pidOutput = flywheelControllerLow.calculate(flywheelSubsystem.getFlywheelRPM(), shooterState.rpmVelocity);
        } else {
            pidOutput = flywheelControllerFar.calculate(flywheelSubsystem.getFlywheelRPM(), shooterState.rpmVelocity);
        }

        BigDecimal KF_PERCENT_FACTOR_FLYWHEEL = new BigDecimal("0.00018082895");
        BigDecimal KF_CONSTANT = new BigDecimal("0.0159208876");

        BigDecimal feedforward = (new BigDecimal(shooterState.rpmVelocity).multiply(KF_PERCENT_FACTOR_FLYWHEEL)).add(KF_CONSTANT);

        double feedForwardedPidOutput = pidOutput + feedforward.doubleValue();

        // Ensure it is never negative
        double positiveMotorOutput = (feedForwardedPidOutput <= 0) ? 0 : feedForwardedPidOutput;
        double clampedPositiveFinalMotorOutput = (positiveMotorOutput > 1) ? 1 : positiveMotorOutput;

        flywheelSubsystem.setPercentSpeed(clampedPositiveFinalMotorOutput);
        flywheelSubsystem.setHoodAngle(shooterState.hoodAngle);

        SmartDashboard.putNumber("Flywheel Output",clampedPositiveFinalMotorOutput);

    }

    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.stopFlywheel();
    }
}
