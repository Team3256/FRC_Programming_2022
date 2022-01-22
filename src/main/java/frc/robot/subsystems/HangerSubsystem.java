package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.HangerConstants.*;

public class HangerSubsystem extends SubsystemBase {
    private final TalonFX hangerMotor;
    private final PneumaticsControlModule pneumaticControl1;
    private final PneumaticsControlModule pneumaticControl2;

    public HangerSubsystem() {
        hangerMotor = new TalonFX(HANGER_TALON_FX_MOTOR_ID);

        pneumaticControl1 = new PneumaticsControlModule(0);
        pneumaticControl2 = new PneumaticsControlModule(5);

        pneumaticControl1.makeDoubleSolenoid(SOLENOID_FORWARD_ONE, SOLENOID_BACKWARD_ONE);
        pneumaticControl2.makeDoubleSolenoid(SOLENOID_FORWARD_TWO, SOLENOID_BACKWARD_TWO);
    }

    public void hangerMotorSpool() {
        hangerMotor.set(ControlMode.Velocity, SPOOL_FORWARD_SPEED);

    }

    public void hangerMotorUnspool() {
        hangerMotor.set(ControlMode.Velocity, SPOOL_BACKWARD_SPEED);

    }

    public void pneumaticFire() {

    }

    public void pneumaticUnfire() {

    }
}
