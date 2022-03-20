package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

/*
    Huge thanks for team254 for giving the inspiration for most of the code in these classes <3
    https://github.com/Team254/FRC-2020-Public/tree/master/src/main/java/com/team254/lib/drivers
*/
public class TalonFXFactory {
    private static TalonConfiguration defaultTalonFXConfig = new TalonConfiguration();
    private static TalonConfiguration defaultFollowerTalonFXConfig = new TalonConfiguration();

    public static TalonFX createTalonFX(int id, String canBus) {
        return createTalonFX(id, defaultTalonFXConfig, canBus);
    }

    public static TalonFX createTalonFX(int id, TalonConfiguration config, String canBus) {
        TalonFX motor = new LazyTalonFX(id, canBus);

        motor.clearMotionProfileHasUnderrun();
        motor.clearMotionProfileTrajectories();
        motor.clearStickyFaults();

        motor.configAllSettings(config.TALONFX_CONFIG);
        motor.configStatorCurrentLimit(config.STATOR_CURRENT_LIMIT);
        motor.configSupplyCurrentLimit(config.SUPPLY_CURRENT_LIMIT);


        motor.config_kP(0, config.PIDF_CONSTANTS.kP);
        motor.config_kI(0, config.PIDF_CONSTANTS.kI);
        motor.config_kD(0, config.PIDF_CONSTANTS.kD);
        motor.config_kF(0, config.PIDF_CONSTANTS.kF);

        motor.setNeutralMode(config.NEUTRAL_MODE);
        motor.setInverted(config.INVERT_TYPE);

        return motor;
    }

    public static TalonFX createFollowerTalonFX(int id, TalonFX master, String canBus) {
        return createFollowerTalonFX(id, master, defaultFollowerTalonFXConfig, canBus);
    }

    public static TalonFX createFollowerTalonFX(int id, TalonFX master, TalonConfiguration config, String canBus) {
        TalonFX motor = createTalonFX(id, config, canBus);
        motor.follow(master);
        return motor;
    }
}
