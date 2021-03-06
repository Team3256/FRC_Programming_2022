package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class LazyTalonFX extends TalonFX {
    double lastSetValue = Double.NaN;
    protected TalonFXControlMode lastSetControlMode = null;

    public LazyTalonFX(int id, String canBus) {
        super(id, canBus);
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

    @Override
    public void neutralOutput() {
        lastSetValue = 0;
        super.neutralOutput();
    }
}