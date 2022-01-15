package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class PIDShooterSubsystem extends PIDSubsystem {
    private final TalonFX m_shooterMotor = new TalonFX(Constants.IDConstants.PID_SHOOTER_MOTOR_ID);

    public PIDShooterSubsystem(PIDController controller, double initialPosition) {
        super(controller, initialPosition);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // leave blank
    }

    public void setSpeed(double velocity) {
        m_shooterMotor.set(ControlMode.Velocity, velocity);
    }

    @Override
    protected double getMeasurement() {
        return 0;
    }
}
