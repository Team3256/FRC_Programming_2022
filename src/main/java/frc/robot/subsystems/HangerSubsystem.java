package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.HangerConstants.*;
import static frc.robot.Constants.IDConstants.*;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
public class HangerSubsystem extends SubsystemBase {
    private final TalonFX masterTalonMotor;
    private final TalonFX followerTalonMotor;
    DoubleSolenoid leftSolenoid;
    DoubleSolenoid rightSolenoid;


    public HangerSubsystem() {
        masterTalonMotor = new TalonFX(MASTER_TALON_ID);
        followerTalonMotor = new TalonFX(FOLLOWER_TALON_ID);

        masterTalonMotor.setInverted(INVERT_MOTOR);

        followerTalonMotor.follow(masterTalonMotor);
        followerTalonMotor.setInverted(InvertType.FollowMaster);

        masterTalonMotor.setNeutralMode(NeutralMode.Brake);
        followerTalonMotor.setNeutralMode(NeutralMode.Brake);

        leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,SOLENOID_LEFT_FORWARD, SOLENOID_LEFT_BACKWARD);
        rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,SOLENOID_RIGHT_FORWARD, SOLENOID_RIGHT_BACKWARD);
    }

    public void extend() {
        masterTalonMotor.set(ControlMode.Position, EXTEND_DISTANCE);
        followerTalonMotor.set(ControlMode.Position, EXTEND_DISTANCE);
    }

    public void retractContinuously() {
        masterTalonMotor.set(ControlMode.PercentOutput, RETRACT_PERCENT);
        followerTalonMotor.set(ControlMode.PercentOutput, RETRACT_PERCENT);
    }

    public void extendPartial() {
        masterTalonMotor.set(ControlMode.Position, PARTIAL_DISTANCE);
        followerTalonMotor.set(ControlMode.Position, PARTIAL_DISTANCE);
    }

    public void pneumaticUpright() {
        leftSolenoid.set(kForward);
        rightSolenoid.set(kForward);
    }

    public void pneumaticSlant() {
        leftSolenoid.set(kReverse);
        rightSolenoid.set(kReverse);
    }

    public boolean isCurrentReached() {
        return masterTalonMotor.getSupplyCurrent() >= CURRENT_LIMIT;
    }

    public void stopMotor() {
        masterTalonMotor.set(ControlMode.PercentOutput, 0);
        followerTalonMotor.set(ControlMode.PercentOutput, 0);
    }

}
