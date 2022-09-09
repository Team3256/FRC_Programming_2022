// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.TalonConfiguration;
import frc.robot.hardware.TalonFXFactory;
import frc.robot.helper.logging.RobotLogger;
import io.github.oblarg.oblog.Loggable;

import java.util.logging.Logger;

import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase implements Loggable {
    private static final RobotLogger logger = new RobotLogger(IntakeSubsystem.class.getCanonicalName());

    private final TalonFX intakeMotor;
    private final DoubleSolenoid leftintakeSolenoid;
    private final DoubleSolenoid rightIntakeSolenoid;

    public IntakeSubsystem() {
        TalonConfiguration config = new TalonConfiguration(NeutralMode.Coast, InvertType.InvertMotorOutput);
        intakeMotor = TalonFXFactory.createTalonFX(INTAKE_MOTOR_ID, config, MANI_CAN_BUS);
        leftintakeSolenoid = new DoubleSolenoid(PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, INTAKE_SOLENOID_LEFT_FORWARD, INTAKE_SOLENOID_LEFT_BACKWARD);
        rightIntakeSolenoid = new DoubleSolenoid(PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, INTAKE_SOLENOID_RIGHT_FORWARD, INTAKE_SOLENOID_RIGHT_BACKWARD);
        logger.info("Intake Initialized");

        off();
    }

    public void forwardOn(){
        logger.info("Intake on");
        extend();
        intakeMotor.set(ControlMode.PercentOutput, INTAKE_FORWARD_SPEED);
    }

    public void intakeDown(){
        logger.info("Intake coming down");
        leftintakeSolenoid.set(DoubleSolenoid.Value.kForward);
        rightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void reverseOn(){
        retract();
        intakeMotor.set(ControlMode.PercentOutput, INTAKE_BACKWARD_SPEED);
    }

    public void outtake(){
        retract();
        intakeMotor.set(ControlMode.PercentOutput, INTAKE_OUTTAKE_SPEED);
    }

    public void off(){
        intakeMotor.neutralOutput();
        retract();
        logger.info("Intake off");
    }
    public void extend(){
        leftintakeSolenoid.set(DoubleSolenoid.Value.kForward);
        rightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
        logger.info("Intake Extended");
    }
    public void retract(){
        leftintakeSolenoid.set(DoubleSolenoid.Value.kReverse);
        rightIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
        logger.info("Intake Retracted");
    }
}
