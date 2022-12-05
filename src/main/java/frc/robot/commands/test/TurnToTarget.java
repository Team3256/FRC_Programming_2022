package frc.robot.commands.test;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveDrive;
import org.photonvision.PhotonCamera;

public class TurnToTarget extends PIDCommand {
    private SwerveDrive swerveDrive;
    private PhotonCamera photonCamera;

    public TurnToTarget(SwerveDrive drive, PhotonCamera photonCamera) {
        super(
                new PIDController(0.1,0,0),
                drive.getPose().getRotation()::getDegrees,
                photonCamera.getLatestResult().getBestTarget().getYaw(),
                output -> drive.drive(new ChassisSpeeds(0,0,output)),
                drive
        );

        getController().enableContinuousInput(-180,180);
        getController().setTolerance(3);
    }



    @Override
    public void initialize() {
        System.out.println("Running Turn To Target Command");
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
