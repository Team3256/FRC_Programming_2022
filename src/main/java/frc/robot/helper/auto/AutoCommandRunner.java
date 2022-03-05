package frc.robot.helper.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import static frc.robot.Constants.AutoConstants.COMMAND_MARKER_THRESHOLD;

public class AutoCommandRunner {
    private List<AutoCommandMarker> commandMarkers;
    private List<AutoCommandMarker> startedCommandMarkers;
    private Pose2d lastPose;

    public AutoCommandRunner(List<AutoCommandMarker> markers) {
        commandMarkers = markers;
    }

    public void execute(Pose2d currentPose) {
        Iterator<AutoCommandMarker> commandMarkerIterator = commandMarkers.iterator();
        Iterator<AutoCommandMarker> startedCommandMarkerIterator = startedCommandMarkers.iterator();
        if (lastPose == null) {
            while (commandMarkerIterator.hasNext()) {
                AutoCommandMarker autoCommandMarker = commandMarkerIterator.next();
                if (isAtMarker(autoCommandMarker.getMarker(), currentPose)) {
                    autoCommandMarker.getCommand().schedule();
                    startedCommandMarkers.add(autoCommandMarker);
                    commandMarkerIterator.remove();
                }
            }

            while (startedCommandMarkerIterator.hasNext()) { // cancel started commands
                AutoCommandMarker autoCommandMarker = startedCommandMarkerIterator.next();
                if (isAtMarker(autoCommandMarker.getEndingMarker(), currentPose)) {
                    autoCommandMarker.getCommand().cancel();
                    startedCommandMarkerIterator.remove();
                }
            }
        } else {
            while (commandMarkerIterator.hasNext()) {
                AutoCommandMarker autoCommandMarker = commandMarkerIterator.next();
                if (isAtMarker(autoCommandMarker.getMarker(), currentPose, lastPose)) {
                    autoCommandMarker.getCommand().schedule();
                    startedCommandMarkers.add(autoCommandMarker);
                    commandMarkerIterator.remove();
                }
            }

            while (startedCommandMarkerIterator.hasNext()) { // cancel started commands
                AutoCommandMarker autoCommandMarker = startedCommandMarkerIterator.next();
                if (isAtMarker(autoCommandMarker.getEndingMarker(), currentPose, lastPose)) {
                    autoCommandMarker.getCommand().cancel();
                    startedCommandMarkerIterator.remove();
                }
            }
        }
        lastPose = currentPose;
    }

    public void end() {
        Iterator<AutoCommandMarker> startedCommandMarkerIterator = startedCommandMarkers.iterator();
        while (startedCommandMarkerIterator.hasNext()) {
            startedCommandMarkerIterator.next().getCommand().cancel();
        }
    }

    private boolean isAtMarker(Translation2d marker, Pose2d currentPose) {
        if (marker == null || currentPose == null) {
            return false;
        }

        double distance = marker.getDistance(currentPose.getTranslation());
        return distance < COMMAND_MARKER_THRESHOLD;
    }

    private boolean isAtMarker(Translation2d marker, Pose2d currentPose, Pose2d lastPose) {
        if (marker == null || currentPose == null || lastPose == null) {
            return false;
        }

        Translation2d currentTranslation = currentPose.getTranslation();
        Translation2d lastTranslation = lastPose.getTranslation();

        double distanceBetweenTrajectoryPoses = currentTranslation.getDistance(lastTranslation);
        double distanceBetweenLastPose = lastTranslation.getDistance(marker);
        double distanceBetweenCurrentPose = currentTranslation.getDistance(marker);

        // essentially finding if the marker is in between the 2 trajectories
        // by kinda creating an ellipse/box while allowing for some tolerance
        return distanceBetweenCurrentPose < distanceBetweenTrajectoryPoses
                && distanceBetweenLastPose < distanceBetweenTrajectoryPoses;
    }
}
