package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/*
    Huge thanks for team254 for giving the inspiration for most of the code in these classes <3
    https://github.com/Team254/FRC-2020-Public/tree/master/src/main/java/com/team254/lib/drivers
*/
public class TalonFXFactory {
    private static TalonConfiguration defaultTalonFXConfig = new TalonConfiguration();
    private static TalonConfiguration defaultFollowerTalonFXConfig = new TalonConfiguration();

    public static TalonFX createTalonFX(int id) {
        return createTalonFX(id, defaultTalonFXConfig);
    }

    public static TalonFX createTalonFX(int id, TalonConfiguration config) {
        TalonFX motor = new LazyTalonFX(id);

        motor.clearMotionProfileHasUnderrun();
        motor.clearMotionProfileTrajectories();
        motor.clearStickyFaults();

        motor.config_kP(0, config.PIDF_CONSTANTS.kP); //TODO: change slotIdx if required
        motor.config_kI(0, config.PIDF_CONSTANTS.kI); //TODO: change slotIdx if required
        motor.config_kD(0, config.PIDF_CONSTANTS.kD); //TODO: change slotIdx if required
        motor.config_kF(0, config.PIDF_CONSTANTS.kF); //TODO: change slotIdx if required

        motor.setNeutralMode(config.NEUTRAL_MODE);
        motor.setInverted(config.INVERT_TYPE);
        motor.configAllSettings(config.TALONFX_CONFIG);

        return motor;
    }

    public static TalonFX createFollowerTalonFX(int id, int master) {
        return createFollowerTalonFX(id, master, defaultFollowerTalonFXConfig);
    }

    public static TalonFX createFollowerTalonFX(int id, int master, TalonConfiguration config) {
        TalonFX motor = createTalonFX(id, config);
        motor.set(ControlMode.Follower, master);
        return motor;
    }
}
