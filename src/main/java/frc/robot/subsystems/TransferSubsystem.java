// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hardware.MuxedColorSensor;
import frc.robot.hardware.TalonFXFactory;
import frc.robot.commands.transfer.TransferIndexForward;
import frc.robot.commands.transfer.TransferOff;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helper.BallColor;

import java.util.LinkedList;
import java.util.logging.Logger;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.*;
import static frc.robot.Constants.CANdleConstants.BALL_PATTERN;
import static frc.robot.Constants.IDConstants;

import static frc.robot.Constants.TransferConstants;
import static frc.robot.Constants.TransferConstants.*;
import static frc.robot.helper.BallColor.*;

public class TransferSubsystem extends SubsystemBase {
    private static final Logger logger = Logger.getLogger(TransferSubsystem.class.getCanonicalName());

    private final TalonFX transferMotor;
    private final DigitalInput transferStartIRSensor;
    private final DigitalInput transferStopIRSensor;
    private final DigitalInput transferEndIRSensor;

    // Counts how many times what color is being detected
    // To avoid single bad reading skewing data
    private int blueColorCountVote = 0;
    private int redColorCountVote = 0;

    private boolean isDetectingBallColor = false;
    private boolean isReversed = false;

    DriverStation.Alliance alliance;

    // Linked list for FIFO queue
    LinkedList<BallColor> ballColorIndex = new LinkedList<>();


    private double currentBallCount;

    public TransferSubsystem() {
        transferMotor = TalonFXFactory.createTalonFX(IDConstants.FEEDER_MOTOR_ID);
      
        transferStartIRSensor = new DigitalInput(IDConstants.IR_TRANSFER_BEGINNING_CHANNEL);
        transferStopIRSensor = new DigitalInput(IDConstants.IR_TRANSFER_MIDDLE_CHANNEL);
        transferEndIRSensor = new DigitalInput(IDConstants.IR_TRANSFER_END_CHANNEL);

        SmartDashboard.setDefaultNumber("Starting Ball Count", STARTING_BALL_COUNT);
        currentBallCount = SmartDashboard.getNumber("Starting Ball Count", TransferConstants.STARTING_BALL_COUNT);

        alliance = DriverStation.getAlliance();
        if (alliance == DriverStation.Alliance.Invalid)
            logger.warning("Alliance Info Invalid!");

        // Add Same Color of Ball when starting out
        if (currentBallCount == 1)
            addBallToIndex(DriverStation.getAlliance() == Blue ? BLUE : RED);

        transferIndexSetup();
        logger.info("Transfer Initialized");
        logger.config("Starting Ball Count Initialized to: " + currentBallCount);
    }

    public void forward(){
        isReversed = false;
        transferMotor.set(TalonFXControlMode.PercentOutput, TransferConstants.DEFAULT_TRANSFER_SPEED);
        logger.info("Transfer On");
    }

    public void manualReverse(){
        isReversed = true;
        transferMotor.set(TalonFXControlMode.PercentOutput, MANUAL_REVERSE_TRANSFER_SPEED);
        logger.info("Transfer Manually Reversed");
    }

    public void off(){
        isReversed = false;
        transferMotor.neutralOutput();
        logger.info("Transfer Off");
    }

    /**
     * @return Returns whether the IR sensor at the front of the transfer's line of sight is broken.
     */
    public boolean isTransferStartIRBroken() {
        return transferStartIRSensor.get();
    }

    /**
     * @return Returns whether the IR sensor in the middle of the transfer's line of sight is broken,
     * which signifies when the ball should stop.
     */
    public boolean isTransferStopIRBroken() {
        return transferStopIRSensor.get();
    }

    /**
     * @return Returns whether the IR sensor at the end of the transfer's line of sight is broken,
     * which signifies when the ball leaves the transfer via the shooter
     */
    public boolean isTransferEndIRBroken() {
        return transferEndIRSensor.get();
    }

    public boolean isFull(){
        return currentBallCount >= TransferConstants.MAX_BALL_COUNT;
    }

    public void transferIndexSetup(){

        // Starts Index / Counting Process when First Detecting Ball
        new Trigger(this::isTransferStartIRBroken).and(new Trigger(()->!isReversed))
                .whenActive(new ParallelCommandGroup(
                        new InstantCommand(this::ballIndexStart),
                        new TransferIndexForward(this)
                ));


        // Stop Running Transfer when past end mark, also evaluates color
        new Trigger(this::isTransferStopIRBroken).and(new Trigger(()->!isReversed))
                .whenInactive(new ParallelCommandGroup(
                    new InstantCommand(this::ballIndexEnd),
                    new TransferOff(this)));

        // Subtract Balls shot out of shooter
        new Trigger(this::isTransferEndIRBroken).and(new Trigger(()->!isReversed))
                .whenInactive(new InstantCommand(this::removeShotBallFromIndex));

        // When Reversed, Subtract Balls that leave
        new Trigger(this::isTransferStartIRBroken).and(new Trigger(()-> isReversed))
                .whenActive(new InstantCommand(this::removeBallEjectedOutOfIntake));
    }

    private void ballIndexStart(){
        redColorCountVote = 0;
        blueColorCountVote = 0;

        currentBallCount++;

        if(isFull()){
            logger.info("Indexing while Transfer is full!");
        }
    }

    /**
     * Runs when Ball is between Transfer Start + Stop sensors to detect ball color
     */
    private void ballColorSamplingPeriodic() {
        double proximity = MuxedColorSensor.getInstance().getBallSensorProximity();

        if (proximity >= MIN_BALL_COLOR_PROXIMITY){
            BallColor ballColor = MuxedColorSensor.getInstance().ballSensorDetection();
            if (ballColor == BLUE)
                blueColorCountVote++;
            else if (ballColor == RED)
                redColorCountVote++;
        }
    }

    private void ballIndexEnd(){
        isDetectingBallColor = false;

        if (redColorCountVote == 0 && blueColorCountVote == 0)
            logger.warning("No Ball Detected in Index!");

        else if (redColorCountVote == blueColorCountVote)
            logger.warning("Blue and Red Ball Counts are the same!\nCount: " + redColorCountVote);

        else if (redColorCountVote > blueColorCountVote) {
            addBallToIndex(RED);
        }
        else {
           addBallToIndex(BLUE);
        }
    }

    private void addBallToIndex(BallColor ballColor){
        logger.info("Ball Indexed Into Transfer");

        if (ballColor == RED && alliance == Blue)
            wrongBallColorDetected(ballColor);

        if (ballColor == BLUE && alliance == Red)
            wrongBallColorDetected(ballColor);

        // Keep 2nd Ball in 2nd Place, if there is one
        if (ballColorIndex.get(0) == NONE){
            ballColorIndex.set(0, ballColor);
        } else {
            ballColorIndex.addFirst(ballColor);
        }

        // Remove extra NONEs
        if (ballColorIndex.getLast() == NONE)
            ballColorIndex.removeLast();

        updateBallLEDPattern();
    }

    private void removeShotBallFromIndex(){
        logger.info("Ball Leaving Transfer by Shooting");

        currentBallCount--;

        if (ballColorIndex.getLast() == NONE)
            logger.warning("No Ball At end of index!");

        ballColorIndex.removeLast();
        ballColorIndex.addFirst(NONE);

        updateBallLEDPattern();
    }

    private void removeBallEjectedOutOfIntake(){
        logger.info("Ball Leaving Transfer out of Intake");

        currentBallCount--;

        if (ballColorIndex.get(0) == NONE){
            // Only Ball is Indexed 2nd, Removes the NONE and the Ball
            ballColorIndex.remove(0);
            ballColorIndex.remove(1);
        } else {
            ballColorIndex.removeFirst();
        }
    }

    private void updateBallLEDPattern(){
        BallColor firstBallColor = ballColorIndex.size() < 1 ? NONE : ballColorIndex.get(0);
        BallColor secondBallColor = ballColorIndex.size() < 2 ? NONE : ballColorIndex.get(1);

        BALL_PATTERN.update(firstBallColor, secondBallColor);
    }

    private void wrongBallColorDetected(BallColor ballColorDetected){
        logger.warning("Intaked Wrong Ball Color! " +
                "(Alliance: " + alliance +
                ", Ball Color Intaken: " + ballColorDetected +
                ")");
    }

    @Override
    public void periodic() {
        if (isDetectingBallColor)
            ballColorSamplingPeriodic();
    }
}
