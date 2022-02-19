// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private boolean firstBreak;

    public FeederSubsystem() {
        feederMotor = new TalonFX(IDConstants.FEEDER_MOTOR_ID);
        feederStartIRSensor = new DigitalInput(IDConstants.START_CHANNEL);
        feederStopIRSensor = new DigitalInput(IDConstants.STOP_CHANNEL);
        feederEndIRSensor = new DigitalInput(IDConstants.END_CHANNEL);

        SmartDashboard.setDefaultNumber("Starting Ball Count", FeederConstants.MAX_BALL_COUNT);
        currentBallCount = SmartDashboard.getNumber("Starting Ball Count", FeederConstants.MAX_BALL_COUNT);
        firstBreak = true;

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
        if(firstBreak && isFeederStartIRBroken()){
            if(isFull()){
                logger.info("Feeder is full");
            }
            currentBallCount++;
            firstBreak = false;
            new FeederOn(this).schedule();
        }
        else if(isFeederStopIRBroken()){
            new FeederOff(this).schedule();
            firstBreak = true;
        }
        if(isFeederEndIRBroken()){
            currentBallCount--;
        }
    }
    public void off(){
        feederMotor.set(TalonFXControlMode.PercentOutput, 0);
        logger.info("Feeder Off");
    }

    @Override
    public void periodic() {
        super.periodic();
        transferIndex();
        CommandScheduler.getInstance().run();
    }
}
