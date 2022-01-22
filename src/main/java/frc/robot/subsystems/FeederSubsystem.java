// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
    private final TalonFX feederMotor;
    public FeederSubsystem() {
        feederMotor = new TalonFX(FeederConstants.FEEDER_MOTOR_ID);
    }
    public void on(){
        feederMotor.set(TalonFXControlMode.Current, FeederConstants.DEFAULT_FEEDER_SPEED);
    }
    public void off(){
        feederMotor.set(TalonFXControlMode.Current, 0);
    }
}
