package frc.robot.helper;


import edu.wpi.first.math.geometry.Rotation2d;

public class UniformThetaSupplier implements ThetaSupplier {
    private double trajectoryDuration;
    private Rotation2d desiredRotation;
    private double proportion;

    public UniformThetaSupplier(double trajectoryDuration, Rotation2d desiredRotation, double proportion) {
        this.trajectoryDuration = trajectoryDuration;
        this.desiredRotation = desiredRotation;
        this.proportion = proportion;
    }

    public UniformThetaSupplier(double trajectoryDuration) {
        this.trajectoryDuration = trajectoryDuration;
        this.desiredRotation = new Rotation2d();
        this.proportion = 1;
    }

    public UniformThetaSupplier(Rotation2d desiredRotation, double proportion) {
        this.trajectoryDuration = 10; // SHOULD GET CHANGED BY TRAJECTORY FACTORY
        this.desiredRotation = desiredRotation;
        this.proportion = proportion;
    }

    public void setTrajectoryDuration(double duration) {
        this.trajectoryDuration = duration;
    }

    public Rotation2d rotationSupply(double now) {
        return new Rotation2d((this.desiredRotation.getRadians() >= 0 ? 1 : -1) * Math.min(
                Math.abs(
                        this.desiredRotation.getRadians() * (now/(this.trajectoryDuration * proportion))),
                Math.abs(
                        this.desiredRotation.getRadians())
        ));
    }
}

