// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.TalonFXFactory;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.transfer.TransferOn;
import frc.robot.commands.transfer.TransferOff;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.logging.Logger;

import static frc.robot.Constants.IDConstants;

import static frc.robot.Constants.TransferConstants;
import static frc.robot.Constants.TransferConstants.STARTING_BALL_COUNT;

public class TransferSubsystem extends SubsystemBase {
    private static final Logger logger = Logger.getLogger(TransferSubsystem.class.getCanonicalName());

    private final TalonFX transferMotor;
    private final DigitalInput transferStartIRSensor;
    private final DigitalInput transferStopIRSensor;
    private final DigitalInput transferEndIRSensor;


    private double currentBallCount;

    public TransferSubsystem() {
        transferMotor = TalonFXFactory.createTalonFX(IDConstants.FEEDER_MOTOR_ID);
      
        transferStartIRSensor = new DigitalInput(IDConstants.IR_TRANSFER_BEGINNING_CHANNEL);
        transferStopIRSensor = new DigitalInput(IDConstants.IR_TRANSFER_MIDDLE_CHANNEL);
        transferEndIRSensor = new DigitalInput(IDConstants.IR_TRANSFER_END_CHANNEL);

        SmartDashboard.setDefaultNumber("Starting Ball Count", STARTING_BALL_COUNT);
        currentBallCount = SmartDashboard.getNumber("Starting Ball Count", TransferConstants.STARTING_BALL_COUNT);

        transferIndexSetup();
        logger.info("Transfer Initialized");
        logger.config("Starting Ball Count Initialized to: " + currentBallCount);
    }
    public void on(){
        transferMotor.set(TalonFXControlMode.PercentOutput, TransferConstants.DEFAULT_TRANSFER_SPEED);
        logger.info("Transfer On");
    }

    public boolean isTransferStartIRBroken() {
        return transferStartIRSensor.get();
    }

    public boolean isTransferStopIRBroken() {
        return transferStopIRSensor.get();
    }

    public boolean isTransferEndIRBroken() {
        return transferEndIRSensor.get();
    }

    public boolean isFull(){
        return currentBallCount == TransferConstants.MAX_BALL_COUNT;
    }

    public void transferIndexSetup(){
        new Button(this::isTransferStartIRBroken)
                .whenPressed(new ParallelCommandGroup(
                                new InstantCommand(()->{
                                    currentBallCount++;
                                    if(isFull()){
                                        logger.info("Transfer is full");
                                    }

                                }),
                                new TransferOn(this)
                ));

        new Button(this::isTransferStopIRBroken).whenReleased(new TransferOff(this));

        new Button(this::isTransferEndIRBroken).whenActive(new InstantCommand(() -> currentBallCount--));
    }

    public void off(){
        transferMotor.set(TalonFXControlMode.PercentOutput, 0);
        logger.info("Transfer Off");
    }
}
