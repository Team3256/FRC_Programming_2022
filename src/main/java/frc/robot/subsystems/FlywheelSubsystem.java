package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.text.DecimalFormat;

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

    public void autoAim(double distance, double angleEntry) {
        ShooterState ikShooterState = ballInverseKinematics(distance, angleEntry);

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

    public void setHoodAngle(double hoodAngle) {
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

    private ShooterState ballInverseKinematics(double distance, double angleEntry) {
        double deltaHeight = UPPER_HUB_AIMING_HEIGHT - SHOOTER_HEIGHT;
        double distToAimPoint = RADIUS_UPPER_HUB + distance;

        double tangentEntryAngle = Math.tan(angleEntry);
        double fourDistHeightTangent = 4 * distToAimPoint * deltaHeight * tangentEntryAngle;
        double distanceToAimSquare = Math.pow(distToAimPoint, 2);
        double deltaHeightSquare = Math.pow(angleEntry, 2);
        double tangentAimDistSquare = Math.pow(distToAimPoint * tangentEntryAngle, 2);
        double tangentAimDist = distToAimPoint * tangentEntryAngle;


        double exitAngleTheta = -2 * Math.atan((distToAimPoint -
                Math.sqrt(tangentAimDistSquare + fourDistHeightTangent + distanceToAimSquare + 4*deltaHeightSquare))
                / (tangentAimDist + 2 * deltaHeight));
        double velocity = 0.3 * Math.sqrt(109/2) *
                ((Math.sqrt(tangentAimDistSquare + fourDistHeightTangent + distanceToAimSquare + 4*deltaHeightSquare))
                / Math.sqrt(tangentAimDist + deltaHeight));

        ShooterState shooterState = new ShooterState(velocity, exitAngleTheta);
        return shooterState;
    }

    private void applyShooterState(ShooterState shooterState) {
        shooterState.velocity = ((Math.floor(shooterState.velocity * 100000 * 100000) / 100000) / 100000);

        setSpeed(shooterState.velocity);
        setHoodAngle(shooterState.theta);
    }

    private double getAngularVelocityFromCalibration(double ballVelocity, double ballAngle) {
        // TODO: Get calibration equations
        return ballVelocity;
    }

    private double getHoodValueFromCalibration(double ballVelocity, double ballAngle) {
        // TODO: Get calibration equations
        return ballAngle;
    }

    public double getVelocity() {
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