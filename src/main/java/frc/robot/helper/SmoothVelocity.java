package frc.robot.helper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import static frc.robot.Constants.SwerveConstants.DECELERATION_CONSTANT;

public class SmoothVelocity {

    public static class RobotKinematicState{
        public double acceleration;
        public double velocity;

        public RobotKinematicState(double velocity, double acceleration) {
            this.acceleration = acceleration;
            this.velocity = velocity;
        }
        public RobotKinematicState() {
            this.acceleration = 0;
            this.velocity = 0;
        }
    }


    /**
     * @param targetVelocity in meters
     * @param deltaTime in seconds
     * @return newVelocity in meters
     */
    public static RobotKinematicState smoothVelocity(RobotKinematicState currentRobotKinematicState, double targetVelocity, double deltaTime, double decelerationConstant) {

        double currentAcceleration = (targetVelocity - currentRobotKinematicState.velocity)/deltaTime;

        // Let it accelerate up fully with no modifications
        // Check if Vel / Acc are same, if so, it is speeding up
        if (Math.signum(currentAcceleration) == Math.signum(currentRobotKinematicState.velocity) ||
                currentRobotKinematicState.velocity == 0 ||
                currentAcceleration == 0)
            return new RobotKinematicState(targetVelocity, 0); // Assume Robot Goes to Set point Instantly


        // If Crosses 0, then just decelerate to zero, then accelerate back up to the speed in the opposite direction.
        if (Math.signum(targetVelocity) != Math.signum(currentRobotKinematicState.velocity))
            targetVelocity = 0;

        double positiveUpdatedAcceleration =
                Math.abs(Math.abs(currentRobotKinematicState.acceleration) - decelerationConstant * deltaTime);

        if (positiveUpdatedAcceleration < 0)
            positiveUpdatedAcceleration = 0;

        double updatedAcceleration = positiveUpdatedAcceleration * Math.signum(currentAcceleration);

        double adjustedTargetVelocity = currentAcceleration > 0 ? Math.min(
                Math.abs(targetVelocity),
                currentRobotKinematicState.velocity + (updatedAcceleration * deltaTime)
        ) : Math.max(
                Math.abs(targetVelocity),
                currentRobotKinematicState.velocity + (updatedAcceleration * deltaTime)
        );

        SmartDashboard.putNumber("Adjusted Smooth Acc", updatedAcceleration);
        return new RobotKinematicState(adjustedTargetVelocity, updatedAcceleration);
    }
}
