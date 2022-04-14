package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.hardware.Limelight;
import frc.robot.helper.shooter.ShootingWhileMovingHelper;
import frc.robot.subsystems.ShooterSubsystem;

import java.math.BigDecimal;

public class SetShooterWhileMoving extends CommandBase {
    private PIDController flywheelControllerFar;
    private PIDController flywheelControllerLow;
    private ShootingWhileMovingHelper shootingWhileMovingHelper;

    private double targetVelocity = 0;
    private double targetHoodAngle = 0;
    private double alpha = 0.1;

    private ShooterSubsystem shooterSubsystem;

    public SetShooterWhileMoving(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        flywheelControllerFar = new PIDController(0.0005,0,0.000008);
        flywheelControllerLow = new PIDController(0.00025,0,0.000008);

        addRequirements(shooterSubsystem);
    }

    public SetShooterWhileMoving(ShooterSubsystem shooterSubsystem, XboxController operatorController) {
        this(shooterSubsystem);
//        new Button(() -> flywheelSubsystem.isAtSetPoint(targetVelocity)).whenPressed(new WaitAndVibrateCommand(operatorController, 0.5, 0.1));
    }

    @Override
    public void initialize() {
        System.out.println("Velocity PID Ramping Up");
        Limelight.enable();

        this.shootingWhileMovingHelper = new ShootingWhileMovingHelper(() -> Limelight.getRawDistanceToTarget(), )
    }

    @Override
    public void execute() {
        double pidOutput;

        ShootingWhileMovingHelper.ShootingWhileMovingState state = this.shootingWhileMovingHelper.calculate(alpha);

        targetVelocity = shooterSubsystem.getFlywheelRPMFromInterpolator(state.distance);
//        shooterSubsystem.setTargetVelocity(targetVelocity);

        targetHoodAngle = shooterSubsystem.getHoodAngleFromInterpolator(state.distance);

        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Interpolation Target Velocity", targetVelocity);
            SmartDashboard.putNumber("Interpolation Target Hood Angle", targetHoodAngle);
        }

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