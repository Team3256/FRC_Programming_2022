package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PIDShooterSubsystem extends PIDSubsystem {

    public PIDShooterSubsystem(PIDController controller, double initialPosition) {
        super(controller, initialPosition);
    }

    @Override
    protected void useOutput(double output, double setpoint) {

    }

    @Override
    protected double getMeasurement() {
        return 0;
    }
}
