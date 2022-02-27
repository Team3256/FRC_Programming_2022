// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.logging.Logger;

import static frc.robot.Constants.IDConstants.INTAKE_ID;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    private static final Logger logger = Logger.getLogger(IntakeSubsystem.class.getCanonicalName());

    private final CANSparkMax intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(INTAKE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        logger.info("Intake Initialized");
    }

    public void forwardOn(){
        logger.info("Intake on");
        intakeMotor.set(INTAKE_FORWARD_SPEED);
    }

    public void reverseOn(){
        intakeMotor.set(INTAKE_BACKWARD_SPEED                   );
    }

    public void off(){
        intakeMotor.stopMotor();
        logger.info("Intake off");
    }
}
