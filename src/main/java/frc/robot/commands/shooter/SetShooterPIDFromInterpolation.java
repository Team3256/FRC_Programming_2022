package frc.robot.commands.shooter;

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
//        Limelight.enable();
    }

    @Override
    public void execute() {
        double pidOutput;
        if(!shooterSubsystem.isShootingAllBalls()){
            currentDistance = Limelight.getRawDistanceToTarget();
        }

        //if the current ball count > 0 && tranfer forward is on
        if (!isShooting.getAsBoolean() || targetVelocity == 0) { // dont update when shooting because limelight gets blocked by shooting ball
            targetVelocity = shooterSubsystem.getFlywheelRPMFromInterpolator(currentDistance);
            shooterSubsystem.setTargetVelocity(targetVelocity);

            targetHoodAngle = shooterSubsystem.getHoodAngleFromInterpolator(currentDistance);
        }

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
//        Limelight.disable();
    }

}
