package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class FlywheelSubsystem extends SubsystemBase {
    private final TalonFX shooterMotor;
    private double currentTargetSpeed;

    public FlywheelSubsystem() {
        shooterMotor = new TalonFX(PID_SHOOTER_MOTOR_ID);
    }

    /**
     * @param velocity Velocity in (m/s)
     * Flywheel speed is set by integrated PID controller
     */
    public void setSpeed(double velocity) {
        // formula for converting m/s to sensor units/100ms
        currentTargetSpeed = velocity * 0.1 * 2048 *(1/(2*Math.PI*FLYWHEEL_RADIUS));
        shooterMotor.set(ControlMode.Velocity, currentTargetSpeed);
    }

    public void stop() {
        shooterMotor.set(ControlMode.PercentOutput, 0);
    }

    public double getVelocity() {
        double velocityInSensorUnits = shooterMotor.getSensorCollection().getIntegratedSensorVelocity();
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