package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.TalonConfiguration;
import frc.robot.hardware.TalonFXFactory;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.helper.shooter.ShooterState;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    private static final RobotLogger logger = new RobotLogger(ShooterSubsystem.class.getCanonicalName());

    public enum ShooterLocationPreset {
        LAUNCHPAD,
        TARMAC_VERTEX,
    }

    private final TalonFX masterLeftShooterMotor;

    private final TalonFX hoodAngleMotor;
    private final DigitalInput limitSwitch;
    private ShooterLocationPreset shooterLocationPreset = ShooterLocationPreset.TARMAC_VERTEX;

    public ShooterSubsystem() {
        TalonConfiguration MASTER_CONFIG = new TalonConfiguration();
        MASTER_CONFIG.NEUTRAL_MODE = NeutralMode.Coast;
        MASTER_CONFIG.INVERT_TYPE = InvertType.InvertMotorOutput;
        MASTER_CONFIG.PIDF_CONSTANTS = new TalonConfiguration.TalonFXPIDFConfig(
                SHOOTER_MASTER_TALON_PID_P,
                SHOOTER_MASTER_TALON_PID_I,
                SHOOTER_MASTER_TALON_PID_D,
                SHOOTER_MASTER_TALON_PID_F
        );
        masterLeftShooterMotor = TalonFXFactory.createTalonFX(
                PID_SHOOTER_MOTOR_ID_LEFT,
                MASTER_CONFIG,
                MANI_CAN_BUS
        );


        TalonConfiguration hoodConfig = new TalonConfiguration(new TalonConfiguration.TalonFXPIDFConfig(1,0,10,0), InvertType.None, NeutralMode.Brake);

        hoodAngleMotor = TalonFXFactory.createTalonFX(HOOD_MOTOR_ID, hoodConfig, MANI_CAN_BUS);
        limitSwitch = new DigitalInput(HOOD_LIMITSWITCH_CHANNEL);

        logger.info("Flywheel Initialized");
    }

    /**
     * @param percent Velocity from min to max as percent from xbox controller (0% - 100%)
     * Flywheel speed is set by integrated get controller
     */
    public void setPercentSpeed(double percent) {
        masterLeftShooterMotor.set(ControlMode.PercentOutput, percent);
    }

    /**
     * @param hoodAngle motor units
     * motor moves to hoodAngle position
     */
    public void setHoodAngle(double hoodAngle) {
        hoodAngleMotor.set(ControlMode.Position, hoodAngle);
    }
    /**
     * stops the hood motor
     */
    public void stopHood(){
        hoodAngleMotor.neutralOutput();
    }
    /**
     * reverses the hood for zeroing the hood motor
     */
    public void hoodSlowReverse(){
        System.out.println("Slow Reverse Hood");
        hoodAngleMotor.set(ControlMode.PercentOutput, HOOD_SLOW_REVERSE_PERCENT);
    }
    /**
     * zeros the hood motor sensor
     */
    public void zeroHoodMotorSensor(){
        hoodAngleMotor.setSelectedSensorPosition(0);
    }
    /**
     * checks if limit switch is pressed
     */
    public boolean isHoodLimitSwitchPressed(){
        return !limitSwitch.get();
    }
    /**
     * Disables powers to flywheel motor, motors change to neutral/coast mode
     */
    public void slowFlywheel(){
        setPercentSpeed(SHOOTER_SLOW_PERCENT);
    }

    public void stopFlywheel() {
        masterLeftShooterMotor.neutralOutput();
    }

    public void setShooterLocationPreset(ShooterLocationPreset preset) {
        SmartDashboard.putString("Shooter Preset: ", preset.toString());
        logger.info("Shooter Preset Changed to " + preset);
        this.shooterLocationPreset = preset;
    }

    /*
     * Confirms if velocity is within margin of set point
     */

    public boolean isAtSetPoint(double setpoint) {
        double velocity = -getFlywheelRPM();

        return (velocity <= setpoint + SET_POINT_ERROR_MARGIN*setpoint) &&
                (velocity >= setpoint - SET_POINT_ERROR_MARGIN*setpoint);
    }
    public boolean isAtSetPoint(DoubleSupplier setpoint) {
        double velocity = -getFlywheelRPM();

        return (velocity <= setpoint.getAsDouble() + SET_POINT_ERROR_MARGIN*setpoint.getAsDouble()) &&
                (velocity >= setpoint.getAsDouble() - SET_POINT_ERROR_MARGIN*setpoint.getAsDouble());
    }

    public static double fromSuToRPM(double su){
        return su  * (10 * 60) / 2048;
    }


    public double getFlywheelRPM(){
        return this.fromSuToRPM(masterLeftShooterMotor.getSelectedSensorVelocity());

    }

    public ShooterState getFlywheelShooterStateFromPreset(){
        return ALL_SHOOTER_PRESETS.get(shooterLocationPreset).shooterState;
    }
    @Override
    public void periodic() {
        NetworkTableInstance.getDefault().getTable("Debug").getEntry("HOOD Limit").setBoolean( this.isHoodLimitSwitchPressed());
    }
}