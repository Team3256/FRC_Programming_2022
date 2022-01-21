// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IDConstants.INTAKE_ID;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(INTAKE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    }
    public void on(int speed){
        intakeMotor.set(speed);
    }
    public void off(){
        intakeMotor.stopMotor();
    }
}
