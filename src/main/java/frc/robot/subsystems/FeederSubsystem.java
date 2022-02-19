// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.feeder.FeederOn;
import frc.robot.commands.feeder.FeederOff;
import frc.robot.hardware.IRSensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.logging.Logger;

import static frc.robot.Constants.IDConstants;
import static frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
    private static final Logger logger = Logger.getLogger(FeederSubsystem.class.getCanonicalName());

    private final TalonFX feederMotor;
    private IRSensors irSensors;

    double currentBallCount;

    public FeederSubsystem() {
        feederMotor = new TalonFX(IDConstants.FEEDER_MOTOR_ID);
        irSensors = IRSensors.getInstance();

        SmartDashboard.setDefaultNumber("Starting Ball Count", FeederConstants.MAX_BALL_COUNT);
        currentBallCount = SmartDashboard.getNumber("Starting Ball Count", FeederConstants.MAX_BALL_COUNT);

        logger.info("Feeder Initialized");
    }
    public void on(){
        feederMotor.set(TalonFXControlMode.PercentOutput, FeederConstants.DEFAULT_FEEDER_SPEED);
        logger.info("Feeder On");
    }

    public boolean isFull(){
        return currentBallCount == FeederConstants.MAX_BALL_COUNT;
    }

    public void transferIndex(){
        if(irSensors.isFeederStartIRBroken()){
            currentBallCount++;
            new FeederOn(this);
            if(isFull()){
                logger.info("Feeder is full");
            }
        }
        else if(irSensors.isFeederStopIRBroken()){
            new FeederOff(this);
        }
        if(irSensors.isFeederEndIRBroken()){
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
    }
}
