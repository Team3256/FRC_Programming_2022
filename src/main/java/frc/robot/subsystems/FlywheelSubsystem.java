package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class FlywheelSubsystem extends SubsystemBase {
    private final TalonFX masterShooterMotor;
    private final TalonFX followerShooterMotor;
    private double currentTargetSpeed;

    public FlywheelSubsystem() {
        masterShooterMotor = new TalonFX(PID_SHOOTER_MOTOR_ID_0);
        followerShooterMotor = new TalonFX(PID_SHOOTER_MOTOR_ID_1);

        masterShooterMotor.setInverted(InvertType.InvertMotorOutput);
        followerShooterMotor.setInverted(InvertType.None);

        followerShooterMotor.follow(masterShooterMotor);

        followerShooterMotor.setNeutralMode(NeutralMode.Coast);
        masterShooterMotor.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * @param velocity Angular Velocity in (rev/s)
     * Flywheel speed is set by integrated PID controller
     */
    public void setSpeed(double velocity) {
        // formula for converting m/s to sensor units/100ms
        currentTargetSpeed = velocity * 204.8; // rev/s * 1s/10 (100ms) * 2048su/1rev
        masterShooterMotor.set(ControlMode.Velocity, currentTargetSpeed);
    }

    public void setPercentSpeed(double percent) {
        masterShooterMotor.set(ControlMode.PercentOutput, percent);
    }

    public void stop() {
        masterShooterMotor.neutralOutput();
    }

    public double getVelocity() {
        double velocityInSensorUnits = masterShooterMotor.getSensorCollection().getIntegratedSensorVelocity();
        return velocityInSensorUnits  * 10 / 2048;
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