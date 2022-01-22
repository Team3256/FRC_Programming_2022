package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends PIDSubsystem {
    private final TalonFX turretMotor = new TalonFX(34);

    public TurretSubsystem() {
        super(new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD));
    }

    @Override
    protected void useOutput(double v, double v1) {
        
    }

    @Override
    protected double getMeasurement() {
        // limelight tx
        return 0;
    }
}