package frc.robot.helper;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Constants.IDConstants;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.subsystems.HangerSubsystem;

import java.util.logging.Logger;

import static frc.robot.Constants.IDConstants.*;

/**
 * A Class to test if CAN devices are online
 */
public class CANTest {
    private static final RobotLogger logger = new RobotLogger(CANTest.class.getCanonicalName());



    /**
     * Main method to test all CAN devices except PCM.
     */
    public static void test() {
        boolean noErrors = testTalonFX() &&
                testSparkMax() &&
                testPDP() &&
                testPigeon();

        if (noErrors) {
            logger.info("All CAN Devices Online");
            System.out.println("All CAN Devices Online");
        } else {
            logger.warning("CAN Errors Exist");
        }
    }

    /**
     * Helper method to test PDP
     *
     * @return Returns whether the PDP is online
     */
    private static boolean testPDP() {
        PowerDistribution pdp = new PowerDistribution();
        double voltage = pdp.getVoltage();
        if (voltage == 0) {
            logger.warning("PDP Offline");
            return false;
        }
        return true;
    }

    /**
     * Helper method to test multiple TalonFX motors
     *
     * @return Returns whether all the TalonFXs are online
     */
    private static boolean testTalonFX() {
        boolean isGood = true;
        for (int id : TALON_FX_IDS) {
            TalonFX talon = new TalonFX(id);
            double temp = talon.getTemperature();
            if (temp == 0) {
                isGood = false;
                logger.warning("Talon FX ID " + id + " Offline");
            }
        }
        return isGood;
    }

    /**
     * Helper method to test the Pigeon
     *
     * @return Returns whether the Pigeon is online
     */
    private static boolean testPigeon() {
        PigeonIMU pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
        double temp = pigeon.getTemp();
        if (temp == 0) {
            logger.warning("Pigeon is Offline");
            return false;
        }
        return true;
    }

    /**
     * @return Returns whether all the SparkMaxes are online
     */
    private static boolean testSparkMax() {
        boolean isGood = true;
        for (int id : SPARK_MAX_IDS) {
            CANSparkMax sparkMax = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.fromId(0));
            double temp = sparkMax.getMotorTemperature();
            if (temp == 0) {
                isGood = false;
                logger.warning("SparkMax " + id + " offline");
            }
        }
        return isGood;
    }
}
