package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class FlywheelSubsystem extends SubsystemBase {
    private final TalonFX shooterMotor0;
    private final TalonFX shooterMotor1;
    private double currentTargetSpeed;

    public FlywheelSubsystem() {
        shooterMotor0 = new TalonFX(PID_SHOOTER_MOTOR_ID_0);
        shooterMotor1 = new TalonFX(PID_SHOOTER_MOTOR_ID_1);

        shooterMotor0.setInverted(InvertType.InvertMotorOutput);
        shooterMotor1.setInverted(InvertType.None);
    }

    /**
     * @param velocity Velocity in (m/s)
     * Flywheel speed is set by integrated PID controller
     */
    public void setSpeed(double velocity) {
        // formula for converting m/s to sensor units/100ms
        currentTargetSpeed = velocity * 0.1 * 2048 *(1/(2*Math.PI*FLYWHEEL_RADIUS));
        shooterMotor0.set(ControlMode.Velocity, currentTargetSpeed);
        shooterMotor1.set(ControlMode.Velocity, currentTargetSpeed);
    }

    public void setPercentSpeed(double percent) {
        shooterMotor0.set(ControlMode.PercentOutput, percent);
        shooterMotor1.set(ControlMode.PercentOutput, percent);
    }

    public void stop() {
        shooterMotor0.set(ControlMode.PercentOutput, 0);
        shooterMotor1.set(ControlMode.PercentOutput, 0);
    }

    public double getVelocity() {
        double velocityInSensorUnits = shooterMotor0.getSensorCollection().getIntegratedSensorVelocity();
        return velocityInSensorUnits  * 10 * (1/2048) * 2 * Math.PI * FLYWHEEL_RADIUS;
    }

    public boolean isAtSetPoint() {
        double velocity = getVelocity();

        if ((velocity <= currentTargetSpeed + MARGIN_OF_ERROR_SPEED) && (velocity >= currentTargetSpeed - MARGIN_OF_ERROR_SPEED)) {
            return true;
        } else {
            return false;
        }
    }
}