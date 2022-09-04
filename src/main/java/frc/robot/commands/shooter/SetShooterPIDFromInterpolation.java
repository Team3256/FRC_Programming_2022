package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants;
import frc.robot.commands.WaitAndVibrateCommand;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import jdk.jfr.BooleanFlag;

import java.math.BigDecimal;
import java.util.function.BooleanSupplier;

public class SetShooterPIDFromInterpolation extends CommandBase {
    private PIDController flywheelControllerFar;
    private PIDController flywheelControllerLow;

    private double targetVelocity = 0;
    private double targetHoodAngle = 0;
    private double currentDistance = 0;

    private BigDecimal KF_PERCENT_FACTOR_FLYWHEEL = new BigDecimal("0.00018482895");
    private BigDecimal KF_CONSTANT = new BigDecimal("0.0159208876");


    private ShooterSubsystem shooterSubsystem;
    private BooleanSupplier isShooting;

    public SetShooterPIDFromInterpolation(ShooterSubsystem flywheelSubsystem, BooleanSupplier isShooting) {
        this.shooterSubsystem = flywheelSubsystem;
        this.isShooting = isShooting;

        flywheelControllerFar = new PIDController(0.0005,0,0.000008);
        flywheelControllerLow = new PIDController(0.00025,0,0.000008);
    }

    public SetShooterPIDFromInterpolation(ShooterSubsystem flywheelSubsystem, BooleanSupplier isShooting, XboxController operatorController) {
        this(flywheelSubsystem, isShooting);
        new Button(() -> flywheelSubsystem.isAtSetPoint()).whenPressed(new WaitAndVibrateCommand(operatorController, 0.5, 0.1));
    }

    //shooting all balls
    @Override
    public void initialize() {
        System.out.println("Velocity PID Ramping Up");
        Limelight.enable();
    }

    @Override
    public void execute() {
        double pidOutput;

        currentDistance = Limelight.getRawDistanceToTarget();

        targetVelocity = shooterSubsystem.getFlywheelRPMFromInterpolator(currentDistance) + 15; // PID bad ig
        shooterSubsystem.setTargetVelocity(targetVelocity);

        targetHoodAngle = shooterSubsystem.getHoodAngleFromInterpolator(currentDistance);

        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Interpolation Target Velocity", targetVelocity);
            SmartDashboard.putNumber("Interpolation Target Hood Angle", targetHoodAngle);
        }

        if (targetVelocity < 3500){
            pidOutput = flywheelControllerLow.calculate(shooterSubsystem.getFlywheelRPM(), targetVelocity);
        } else {
            pidOutput = flywheelControllerFar.calculate(shooterSubsystem.getFlywheelRPM(), targetVelocity);
        }

        BigDecimal feedforward = (new BigDecimal(targetVelocity).multiply(KF_PERCENT_FACTOR_FLYWHEEL)).add(KF_CONSTANT);

        double feedForwardedPidOutput = pidOutput + feedforward.doubleValue();

        // Ensure it is never negative or over 100%
        double clampedPositiveFinalMotorOutput = MathUtil.clamp(feedForwardedPidOutput, 0, 1);

        shooterSubsystem.setPercentSpeed(clampedPositiveFinalMotorOutput);
        shooterSubsystem.setHoodAngle(targetHoodAngle);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFlywheel();
//        Limelight.disable();
    }

}
