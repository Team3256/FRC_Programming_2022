package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.helper.Limelight;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends PIDSubsystem {
    private final TalonFX turretMotor = new TalonFX(34);

    public TurretSubsystem() {
        super(new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD));
    }

    //NOTE: enable, disable PID methods built in

    public void manualLeft(){
        turretMotor.set(TalonFXControlMode.PercentOutput, -1*TurretConstants.DEFAULT_TURRET_SPEED);
    }

    public void manualRight(){
        turretMotor.set(TalonFXControlMode.PercentOutput, TurretConstants.DEFAULT_TURRET_SPEED);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        turretMotor.set(TalonFXControlMode.PercentOutput, setpoint+output);
    }

    @Override
    protected double getMeasurement() {
        double ret = Limelight.getTx();
        if (Math.abs(ret) < 0.5) ret = 0;
        return ret;
    }
}