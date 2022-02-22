package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class TalonConfiguration {
    public TalonFXConfiguration TALONFX_CONFIG = new TalonFXConfiguration();
    public TalonFXPIDFConfig PIDF_CONSTANTS = new TalonFXPIDFConfig();
    public InvertType INVERT_TYPE = InvertType.None;
    public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;

    public TalonConfiguration() {}

    public TalonConfiguration(
            TalonFXPIDFConfig talonFXPIDFConfig,
            InvertType invertType,
            NeutralMode neutralMode) {
        this.PIDF_CONSTANTS = talonFXPIDFConfig;
        this.INVERT_TYPE = invertType;
        this.NEUTRAL_MODE = neutralMode;
    }
    public TalonConfiguration(
            TalonFXConfiguration talonFXConfiguration,
            TalonFXPIDFConfig talonFXPIDFConfig,
            InvertType invertType,
            NeutralMode neutralMode) {
        this.TALONFX_CONFIG = talonFXConfiguration;
        this.PIDF_CONSTANTS = talonFXPIDFConfig;
        this.INVERT_TYPE = invertType;
        this.NEUTRAL_MODE = neutralMode;
    }

    public static class TalonFXPIDFConfig {
        double kP = 1;
        double kI = 0;
        double kD = 0;
        double kF = 0;

        public TalonFXPIDFConfig() {}
        public TalonFXPIDFConfig(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }
    }

    public static TalonConfiguration createFollowerConfig(TalonConfiguration masterConfig, InvertType invertType) {
        TalonConfiguration followerConfig = new TalonConfiguration();
        followerConfig.PIDF_CONSTANTS = new TalonFXPIDFConfig(
                masterConfig.PIDF_CONSTANTS.kP,
                masterConfig.PIDF_CONSTANTS.kI,
                masterConfig.PIDF_CONSTANTS.kD,
                masterConfig.PIDF_CONSTANTS.kF
        );
        followerConfig.TALONFX_CONFIG = masterConfig.TALONFX_CONFIG;
        followerConfig.INVERT_TYPE = invertType;
        followerConfig.NEUTRAL_MODE = masterConfig.NEUTRAL_MODE;

        return followerConfig;
    }
}
