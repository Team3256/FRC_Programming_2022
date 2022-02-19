package frc.robot.hardware;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IDConstants;


public class IRSensors {
    private static IRSensors instance;
    private DigitalInput feederStartIRSensor;
    private DigitalInput feederStopIRSensor;
    private DigitalInput feederEndIRSensor;

    public IRSensors() {
        feederStartIRSensor = new DigitalInput(FeederConstants.START_CHANNEL);
        feederStopIRSensor = new DigitalInput(FeederConstants.STOP_CHANNEL);
        feederEndIRSensor = new DigitalInput(FeederConstants.END_CHANNEL);
    }

    public static IRSensors getInstance() {return instance == null ? instance = new IRSensors() : instance;}

    public boolean isFeederStartIRBroken() {
        return feederStartIRSensor.get();
    }

    public boolean isFeederStopIRBroken() {
        return feederStopIRSensor.get();
    }

    public boolean isFeederEndIRBroken() {
        return feederEndIRSensor.get();
    }
}
