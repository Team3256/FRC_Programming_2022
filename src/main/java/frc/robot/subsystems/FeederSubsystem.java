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
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.feeder.FeederOn;
import frc.robot.commands.feeder.FeederOff;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.logging.Logger;

import static frc.robot.Constants.IDConstants;

import static frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
    private static final Logger logger = Logger.getLogger(FeederSubsystem.class.getCanonicalName());

    private final TalonFX feederMotor;
    private DigitalInput feederStartIRSensor;
    private DigitalInput feederStopIRSensor;
    private DigitalInput feederEndIRSensor;


    private double currentBallCount;

    public FeederSubsystem() {
        feederMotor = new TalonFX(IDConstants.FEEDER_MOTOR_ID);
        feederStartIRSensor = new DigitalInput(IDConstants.IR_TRANSFER_BEGINNING_CHANNEL);
        feederStopIRSensor = new DigitalInput(IDConstants.IR_TRANSFER_MIDDLE_CHANNEL);
        feederEndIRSensor = new DigitalInput(IDConstants.IR_TRANSFER_END_CHANNEL);

        SmartDashboard.setDefaultNumber("Starting Ball Count", FeederConstants.MAX_BALL_COUNT);
        currentBallCount = SmartDashboard.getNumber("Starting Ball Count", FeederConstants.MAX_BALL_COUNT);

        transferIndex();
        logger.info("Feeder Initialized");
    }
    public void on(){
        feederMotor.set(TalonFXControlMode.PercentOutput, FeederConstants.DEFAULT_FEEDER_SPEED);
        logger.info("Feeder On");
    }

    public boolean isFeederStartIRBroken() {
        return feederStartIRSensor.get();
    }

    public boolean isFeederStopIRBroken() {
        return feederStopIRSensor.get();
    }

    public boolean isFeederEndIRBroken() {
        return feederEndIRSensor.get();
    }

    public boolean isFull(){
        return currentBallCount == FeederConstants.MAX_BALL_COUNT;
    }

    public void transferIndex(){
        new Button(this::isFeederStartIRBroken)
                .whenPressed(new ParallelCommandGroup(
                                new InstantCommand(()->{
                                    currentBallCount++;
                                    if(isFull()){
                                        logger.info("Feeder is full");
                                    }

                                }),
                                new FeederOn(this)
                ));

        new Button(this::isFeederStopIRBroken).whenReleased(new FeederOff(this));

        new Button(this::isFeederEndIRBroken).whenActive(new InstantCommand(() -> {
            currentBallCount--;
        }));
    }
    public void off(){
        feederMotor.set(TalonFXControlMode.PercentOutput, 0);
        logger.info("Feeder Off");
    }

    @Override
    public void periodic() {
        super.periodic();
        CommandScheduler.getInstance().run();
    }
}
