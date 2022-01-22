package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IDConstants.*;

public class HangerSubsystem extends SubsystemBase {
    private final TalonFX hangerMotor;
    private final PneumaticsControlModule pneumaticControl1;
    private final PneumaticsControlModule pneumaticControl2;

    public HangerSubsystem() {
        hangerMotor = new TalonFX(HANGER_TALON_FX_MOTOR_ID);

        pneumaticControl1 = new PneumaticsControlModule(PNEUMATIC_CONTROL_MODULE_ONE);
        pneumaticControl2 = new PneumaticsControlModule(PNEUMATIC_CONTROL_MODULE_TWO);
    }
}
