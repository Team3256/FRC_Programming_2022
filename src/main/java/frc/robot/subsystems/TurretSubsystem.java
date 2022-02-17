package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.hardware.TalonFXFactory;
import frc.robot.helper.Limelight;

import static frc.robot.Constants.IDConstants;
import static frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends PIDSubsystem {
    private final TalonFX turretMotor;

    /**
     * PID coefficients (kp - proportional ki - integral kd - derivative)
     * setting tolerance which is the air
     */
    public TurretSubsystem() {
        super(new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD));
        getController().setTolerance(TurretConstants.TURRET_TOLERANCE_TX);
        this.setSetpoint(0);
        this.disable();

        turretMotor = TalonFXFactory.createTalonFX(IDConstants.TURRET_ID);
    }

    //NOTE: enable, disable PID methods built in

    public void manualLeft(){
        this.disable();
        turretMotor.set(TalonFXControlMode.PercentOutput, -1*TurretConstants.DEFAULT_TURRET_SPEED);
    }

    public void manualRight(){
        this.disable();
        turretMotor.set(TalonFXControlMode.PercentOutput, TurretConstants.DEFAULT_TURRET_SPEED);
    }
    public void autoAlign(){
        this.enable();
    }

    /**
     * consumes the output of the PID controller, and the current setpoint
     * (which is often useful for computing a feedforward)
     */
    @Override
    protected void useOutput(double output, double setpoint) {
        turretMotor.set(TalonFXControlMode.Current, output);
    }

    /**
     * @return the current measurement of the process variable
     */
    @Override
    protected double getMeasurement() {
        return Limelight.getTx();
    }
}