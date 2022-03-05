// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.TalonFXFactory;
import frc.robot.helper.logging.RobotLogger;

import java.util.logging.Logger;

import static frc.robot.Constants.IDConstants.INTAKE_ID;
import static frc.robot.Constants.IntakeConstants.*;

import static frc.robot.Constants.IDConstants.INTAKE_MOTOR_ID;
import static frc.robot.Constants.IDConstants.MANI_CAN_BUS;

public class IntakeSubsystem extends SubsystemBase {
    private static final RobotLogger logger = new RobotLogger(IntakeSubsystem.class.getCanonicalName());

    private final TalonFX intakeMotor;
    public IntakeSubsystem() {
        intakeMotor = TalonFXFactory.createTalonFX(INTAKE_MOTOR_ID, MANI_CAN_BUS);
        logger.info("Intake Initialized");
    }

    public void forwardOn(){
        logger.info("Intake on");
        intakeMotor.set(ControlMode.PercentOutput, 1);
    }

    public void reverseOn(){
        intakeMotor.set(INTAKE_BACKWARD_SPEED                   );
    }

    public void off(){
        intakeMotor.neutralOutput();
        logger.info("Intake off");
    }
}
