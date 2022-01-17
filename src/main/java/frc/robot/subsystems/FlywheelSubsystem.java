package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class FlywheelSubsystem extends SubsystemBase {
    private final TalonFX masterShooterMotor;
    private final TalonFX followerShooterMotor;

    private final Servo hoodAngleServo;

    private double currentTargetSpeed;

    public FlywheelSubsystem() {
        masterShooterMotor = new TalonFX(PID_SHOOTER_MOTOR_ID_0);
        followerShooterMotor = new TalonFX(PID_SHOOTER_MOTOR_ID_1);

        masterShooterMotor.setInverted(InvertType.InvertMotorOutput);
        followerShooterMotor.setInverted(InvertType.None);

        followerShooterMotor.follow(masterShooterMotor);

        followerShooterMotor.setNeutralMode(NeutralMode.Coast);
        masterShooterMotor.setNeutralMode(NeutralMode.Coast);

        hoodAngleServo = new Servo(HOOD_SERVO_CHANNEL_ID);
    }

    public void autoAim(double distance) {
        ShooterState ikShooterState = ballInverseKinematics(distance);

        ShooterState correctedShooterState = new ShooterState(
                getAngularVelocityFromCalibration(ikShooterState.velocity, ikShooterState.theta),
                getHoodValueFromCalibration(ikShooterState.velocity, ikShooterState.theta));

        applyShooterState(correctedShooterState);
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

    private void setHoodAngle(double hoodAngle) {
        hoodAngleServo.setAngle(hoodAngle);
    }

    public void stop() {
        masterShooterMotor.neutralOutput();
    }

    public boolean isAtSetPoint() {
        double velocity = getVelocity();

        return (velocity <= currentTargetSpeed + MARGIN_OF_ERROR_SPEED) &&
                (velocity >= currentTargetSpeed - MARGIN_OF_ERROR_SPEED);
    }

    private ShooterState ballInverseKinematics(double distance) {
        double theta;

        double aimHeight = UPPER_HUB_HEIGHT + PREFERRED_DISTANCE_FROM_TOP;
        double deltaHeight = aimHeight - SHOOTER_HEIGHT;

        double correctedDistance = distance + (distance * DELTA_DISTANCE_TO_TARGET_FACTOR);
        double correctedAimHeight = aimHeight + (aimHeight * DELTA_AIM_HEIGHT_FACTOR);

        theta = Math.atan((correctedAimHeight + UPPER_HUB_HEIGHT)/(0.5 * correctedDistance));
        double velocity = (correctedDistance * Math.sqrt(CONSTANT_GRAVITY) * 1/(Math.cos(theta))) / ((Math.sqrt(correctedDistance * Math.tan(theta)) - deltaHeight));

        return new ShooterState(velocity, theta);
    }

    private void applyShooterState(ShooterState shooterState) {
        setSpeed(shooterState.velocity);
        setHoodAngle(shooterState.theta);
    }

    private double getAngularVelocityFromCalibration(double ballVelocity, double ballAngle) {
        // TODO: Get calibration equations
        return 0.0;
    }

    private double getHoodValueFromCalibration(double ballVelocity, double ballAngle) {
        // TODO: Get calibration equations
        return 0.0;
    }

    private double getVelocity() {
        double velocityInSensorUnits = masterShooterMotor.getSensorCollection().getIntegratedSensorVelocity();
        return velocityInSensorUnits  * 10 / 2048;
    }
}

class ShooterState {
    public double velocity;
    public double theta;

    public ShooterState(double v, double t) {
        this.velocity = v;
        this.theta = t;
    }
}