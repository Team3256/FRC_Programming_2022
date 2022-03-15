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
    public static RobotKinematicState smoothVelocity(RobotKinematicState currentRobotKinematicState, double targetVelocity, double deltaTime) {

        double currentAcceleration = (targetVelocity - currentRobotKinematicState.velocity)/deltaTime;

        // Let it accelerate up fully with no modifications
        // Check if Vel / Acc are same, if so, it is speeding up
        if (Math.signum(currentAcceleration) == Math.signum(currentRobotKinematicState.velocity) || currentRobotKinematicState.velocity == 0)
            return new RobotKinematicState(targetVelocity, currentAcceleration);

        // Max because we're dealing with Deceleration

        double positiveUpdatedAcceleration =
                Math.abs(currentRobotKinematicState.acceleration) - DECELERATION_CONSTANT * deltaTime;

        if (positiveUpdatedAcceleration < 0)
            positiveUpdatedAcceleration = 0;

        double updatedAcceleration = positiveUpdatedAcceleration * Math.signum(currentAcceleration);

        double adjustedTargetVelocity = currentRobotKinematicState.velocity > 0 ? Math.max(
                Math.abs(targetVelocity),
                (currentRobotKinematicState.velocity + updatedAcceleration * deltaTime)
        ) : Math.min(
                Math.abs(targetVelocity),
                (currentRobotKinematicState.velocity + updatedAcceleration * deltaTime)
        );

        // Checks if Adjusted Velocity is more extreme than Target Velocity, clamp if it is
//        if (adjustedTargetVelocity < 0 && adjustedTargetVelocity < targetVelocity)
//            adjustedTargetVelocity = targetVelocity;
//        else if (adjustedTargetVelocity > 0 && adjustedTargetVelocity > targetVelocity)
//            adjustedTargetVelocity = targetVelocity;


        SmartDashboard.putNumber("Adjusted Smooth Acc", updatedAcceleration);
        return new RobotKinematicState(adjustedTargetVelocity, updatedAcceleration);
    }
}
