// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.logging.Level;
import java.util.logging.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private static final Logger logger = Logger.getLogger(IntakeSubsystem.class.getCanonicalName());

    private final CANSparkMax intakeMotor;
    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(33, CANSparkMaxLowLevel.MotorType.kBrushless);
    }


    public void on(){
        logger.info("Intake on");
        intakeMotor.set(1);
    }
    public void off(){
        intakeMotor.stopMotor();
        logger.info("Intake off");
    }
}
