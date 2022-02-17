package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/*
    Huge thanks for team254 for giving the inspiration for most of the code in these classes <3
    https://github.com/Team254/FRC-2020-Public/tree/master/src/main/java/com/team254/lib/drivers
*/
public class TalonFXFactory {
    public static class Configuration {
        public static class PIDF {
            double kP = 1;
            double kI = 0;
            double kD = 0;
            double kF = 0;
            public PIDF() {}
            public PIDF(double kP, double kI, double kD, double kF) {
                this.kP = kP;
                this.kI = kI;
                this.kD = kD;
                this.kF = kF;
            }
        }

        public TalonFXConfiguration TALONFX_CONFIG = new TalonFXConfiguration();
        public PIDF PIDF_CONSTANTS = new PIDF();
        public InvertType INVERT_TYPE = InvertType.None;
        public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;

        public static Configuration clone(Configuration toClone) {
            Configuration config = new Configuration();
            config.PIDF_CONSTANTS = new PIDF(
                    toClone.PIDF_CONSTANTS.kP,
                    toClone.PIDF_CONSTANTS.kI,
                    toClone.PIDF_CONSTANTS.kD,
                    toClone.PIDF_CONSTANTS.kF
            );
            config.TALONFX_CONFIG = toClone.TALONFX_CONFIG;
            config.INVERT_TYPE = toClone.INVERT_TYPE;
            config.NEUTRAL_MODE = toClone.NEUTRAL_MODE;

            return config;
        }
    }

    private static class LazyTalonFX extends TalonFX {
        double lastSetValue = Double.NaN;
        protected TalonFXControlMode lastSetControlMode = null;

        public LazyTalonFX(int id) {
            super(id);
        }

        @Override
        public void set(TalonFXControlMode mode, double value) {
            // only run when a change happens, reduces CAN usage
            if (value != lastSetValue || mode != lastSetControlMode) {
                lastSetValue = value;
                lastSetControlMode = mode;
                super.set(mode, value);
            }
        }
    }

    private static Configuration defaultTalonFXConfig = new Configuration();
    private static Configuration defaultFollowerTalonFXConfig = new Configuration();

    public static TalonFX createTalonFX(int id) {
        return createTalonFX(id, defaultTalonFXConfig);
    }

    public static TalonFX createTalonFX(int id, Configuration config) {
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

    public static TalonFX createFollowerTalonFX(int id, int master, Configuration config) {
        TalonFX motor = createTalonFX(id, config);
        motor.set(ControlMode.Follower, master);
        return motor;
    }
}
