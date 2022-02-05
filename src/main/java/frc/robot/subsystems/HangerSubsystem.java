package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.naming.ldap.Control;

import static frc.robot.Constants.HangerConstants.*;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
public class HangerSubsystem extends SubsystemBase {
    private final TalonFX hangerMotor;
    DoubleSolenoid leftSolenoid;
    DoubleSolenoid rightSolenoid;

    public HangerSubsystem() {
        hangerMotor = new TalonFX(HANGER_TALON_FX_MOTOR_ID);

        leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,SOLENOID_LEFT_FORWARD, SOLENOID_LEFT_BACKWARD);
        rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,SOLENOID_RIGHT_FORWARD, SOLENOID_RIGHT_BACKWARD);
    }

    public void extend() {
        hangerMotor.set(ControlMode.Position, EXTEND_DISTANCE);

    }

    public void retract() {
        hangerMotor.set(ControlMode.Position, RETRACT_DISTANCE);

    }

    public void extendPartial() {
        hangerMotor.set(ControlMode.Position, PARTIAL_DISTANCE);
    }

    public void pneumaticExtend() {
        leftSolenoid.set(kForward);
        rightSolenoid.set(kForward);
    }

    public void pneumaticRetract() {
        leftSolenoid.set(kReverse);
        rightSolenoid.set(kReverse);
    }
}
