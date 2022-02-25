// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transfer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferSubsystem;

public class TransferOff extends CommandBase {

    private final TransferSubsystem transfer;

    public TransferOff(TransferSubsystem subsystem) {
        transfer = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        transfer.off();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}