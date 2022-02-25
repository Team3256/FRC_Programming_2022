package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.hardware.TalonFXFactory;
import frc.robot.hardware.Limelight;

import static frc.robot.Constants.IDConstants;
import static frc.robot.Constants.TurretConstants;
import static frc.robot.Constants.TurretConstants.GEAR_RATIO;

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
      
        turretMotor.config_kF(0, 0);
        turretMotor.config_kP(0, 0.05);
        turretMotor.config_kI(0, 0.0);
        turretMotor.config_kD(0, 0);
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
    public void ninetyDegreeTurn() {
        turretMotor.setSelectedSensorPosition(0.0);
        turretMotor.set(TalonFXControlMode.Position, 0.25*GEAR_RATIO*(2048));
    }

    public void stop() {
        this.disable();
        turretMotor.set(TalonFXControlMode.Disabled, 1);
    }
    public void autoAlign(){
        this.enable();
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Turret Position", turretMotor.getSelectedSensorPosition());
        if (turretMotor.getControlMode() == TalonFXControlMode.Position.toControlMode())
            SmartDashboard.putNumber("Turret Target", turretMotor.getClosedLoopTarget());
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