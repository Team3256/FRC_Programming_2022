package frc.robot.helper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static frc.robot.Constants.AutoConstants.TRANSLATION_FF;

public class SwerveDriveController {
    PIDController xPositionController;
    PIDController yPositionController;
    ProfiledPIDController thetaController;
    private Pose2d poseTolerance = new Pose2d();
    private Pose2d poseError;
    private Rotation2d rotationError;
    private boolean firstInteration = true;

    public SwerveDriveController(PIDController xPositionController, PIDController yPositionController, ProfiledPIDController thetaController) {
        this.xPositionController = xPositionController;
        this.yPositionController = yPositionController;
        this.thetaController = thetaController;
    }

    public boolean atReference() {
        final Translation2d eTranslate = poseError.getTranslation();
        final Rotation2d eRotate = rotationError;
        final Translation2d tolTranslate = poseTolerance.getTranslation();
        final Rotation2d tolRotate = poseTolerance.getRotation();
        return Math.abs(eTranslate.getX()) < tolTranslate.getX()
                && Math.abs(eTranslate.getY()) < tolTranslate.getY()
                && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
    }

    public ChassisSpeeds calculate(
            Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, Rotation2d angleRef) {
        // If this is the first run, then we need to reset the theta controller to the current pose's
        // heading.
        if (firstInteration) {
            thetaController.reset(currentPose.getRotation().getRadians());
            firstInteration = false;
        }

        // Calculate feedforward velocities (field-relative).
        double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos() * TRANSLATION_FF;
        double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin() * TRANSLATION_FF;
        double thetaFF =
                thetaController.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());

        poseError = poseRef.relativeTo(currentPose);
        rotationError = angleRef.minus(currentPose.getRotation());

        // Calculate feedback velocities (based on position error).
        double xFeedback = xPositionController.calculate(currentPose.getX(), poseRef.getX());
        double yFeedback = yPositionController.calculate(currentPose.getY(), poseRef.getY());

        // Return next output.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
    }
}