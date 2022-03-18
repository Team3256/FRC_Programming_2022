package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hardware.Limelight;
import frc.robot.helper.shooter.ShooterState;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.ShooterConstants.OFFSET_HEIGHT_FACTOR;
import static frc.robot.hardware.Limelight.*;

public class AutoAimShooter extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;
    private HoodSubsystem hoodSubsystem;

    public AutoAimShooter(FlywheelSubsystem flywheelSubsystem, HoodSubsystem hoodSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        addRequirements(flywheelSubsystem, hoodSubsystem);
    }

    @Override
    public void initialize() {
        Limelight.enable();
    }

    @Override
    public void execute() {
        double distance = getRawDistanceToTarget();
        ShooterState ikShooterState = ballInverseKinematics(distance);
        ShooterState correctedShooterState = new ShooterState(
                flywheelSubsystem.getAngularVelocityFromCalibration(ikShooterState.rpmVelocity, ikShooterState.hoodAngle),
                hoodSubsystem.getHoodValueFromCalibration(ikShooterState.rpmVelocity, ikShooterState.hoodAngle));
        flywheelSubsystem.setSpeed(correctedShooterState.rpmVelocity);
        hoodSubsystem.setHoodAngle(correctedShooterState.hoodAngle);
    }

    @Override
    public void end(boolean interrupted) {
        Limelight.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * @param distance distance from target
     * @return ShooterState with velocity and hood angle settings
     */
    private ShooterState ballInverseKinematics(double distance) {
        double angleEntry = ENTRY_ANGLE_INTO_HUB * Math.PI / 180;

        double distToAimPoint = RADIUS_UPPER_HUB + distance;
        distToAimPoint = distToAimPoint +
                DELTA_DISTANCE_TO_TARGET_FACTOR * distToAimPoint + OFFSET_DISTANCE_FACTOR;

        double deltaHeight = UPPER_HUB_AIMING_HEIGHT - SHOOTER_HEIGHT;
        deltaHeight = deltaHeight +
                DELTA_AIM_HEIGHT_FACTOR * distToAimPoint + OFFSET_HEIGHT_FACTOR;

        double tangentEntryAngle = Math.tan(angleEntry);
        double fourDistHeightTangent = 4 * distToAimPoint * deltaHeight * tangentEntryAngle;
        double distanceToAimSquare = Math.pow(distToAimPoint, 2);
        double deltaHeightSquare = Math.pow(deltaHeight, 2);
        double tangentAimDistSquare = Math.pow(distToAimPoint * tangentEntryAngle, 2);
        double tangentAimDist = distToAimPoint * tangentEntryAngle;


        double exitAngleTheta = -2 * Math.atan((distToAimPoint -
                Math.sqrt(tangentAimDistSquare + fourDistHeightTangent + distanceToAimSquare + 4*deltaHeightSquare))
                / (tangentAimDist + 2 * deltaHeight));
        double velocity = 0.3 * Math.sqrt(54.5) *
                ((Math.sqrt(tangentAimDistSquare + fourDistHeightTangent + distanceToAimSquare + 4*deltaHeightSquare))
                        / Math.sqrt(tangentAimDist + deltaHeight));

        return new ShooterState(velocity, exitAngleTheta);
    }
}
