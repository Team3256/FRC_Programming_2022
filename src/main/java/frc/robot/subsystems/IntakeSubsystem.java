// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.TalonFXFactory;
import frc.robot.helper.logging.RobotLogger;

import java.util.logging.Logger;

import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    private static final RobotLogger logger = new RobotLogger(IntakeSubsystem.class.getCanonicalName());

    private final TalonFX intakeMotor;

  private final DoubleSolenoid rightIntakeSolenoid;
    private final DoubleSolenoid leftIntakeSolenoid;

    public IntakeSubsystem() {
        intakeMotor = TalonFXFactory.createTalonFX(INTAKE_MOTOR_ID, MANI_CAN_BUS);

        leftIntakeSolenoid = new DoubleSolenoid(PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, INTAKE_LEFT_SOLENOID_FORWARD, INTAKE_LEFT_SOLENOID_BACKWARD);
        rightIntakeSolenoid = new DoubleSolenoid(PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, INTAKE_RIGHT_SOLENOID_FORWARD, INTAKE_RIGHT_SOLENOID_BACKWARD);

        logger.info("Intake Initialized");

        solenoidRetract();
    }

    public void solenoidExtend(){
        rightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
        leftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void solenoidRetract(){
        rightIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
        leftIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void forwardOn(){
        logger.info("Intake on");
        intakeSolenoid.set(DoubleSolenoid.Value.kForward);
        intakeMotor.set(ControlMode.PercentOutput, INTAKE_FORWARD_SPEED);
    }

    public void reverseOn(){
        intakeMotor.set(ControlMode.PercentOutput, INTAKE_BACKWARD_SPEED);
    }

    public void off(){
        intakeMotor.neutralOutput();
        intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
        logger.info("Intake off");
    }
}
