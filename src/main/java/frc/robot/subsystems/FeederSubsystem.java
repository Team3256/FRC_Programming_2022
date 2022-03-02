// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.TalonFXFactory;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.feeder.FeederOn;
import frc.robot.commands.feeder.FeederOff;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helper.logging.RobotLogger;

import java.util.logging.Logger;

import static frc.robot.Constants.IDConstants;

import static frc.robot.Constants.FeederConstants;
import static frc.robot.Constants.IDConstants.MANI_CAN_BUS;

public class FeederSubsystem extends SubsystemBase {
    private static final RobotLogger logger = new RobotLogger(FeederSubsystem.class.getCanonicalName());

    private final TalonFX feederMotor;
    private DigitalInput feederStartIRSensor;
    private DigitalInput feederStopIRSensor;
    private DigitalInput feederEndIRSensor;


    private double currentBallCount;

    public FeederSubsystem() {
        feederMotor = TalonFXFactory.createTalonFX(IDConstants.FEEDER_MOTOR_ID, MANI_CAN_BUS);
      
        feederStartIRSensor = new DigitalInput(IDConstants.IR_TRANSFER_BEGINNING_CHANNEL);
        feederStopIRSensor = new DigitalInput(IDConstants.IR_TRANSFER_MIDDLE_CHANNEL);
        feederEndIRSensor = new DigitalInput(IDConstants.IR_TRANSFER_END_CHANNEL);

        SmartDashboard.setDefaultNumber("Starting Ball Count", FeederConstants.MAX_BALL_COUNT);
        currentBallCount = SmartDashboard.getNumber("Starting Ball Count", FeederConstants.MAX_BALL_COUNT);

        transferIndexSetup();
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

    public void transferIndexSetup(){
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
        feederMotor.neutralOutput();
        logger.info("Feeder Off");
    }

    @Override
    public void periodic() {
        super.periodic();
        CommandScheduler.getInstance().run();
    }
}
