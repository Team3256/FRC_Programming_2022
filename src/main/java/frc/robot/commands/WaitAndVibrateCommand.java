package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.HangerConstants.PNEUMATIC_WAIT_DURATION;

public class WaitAndVibrateCommand extends CommandBase {
    protected Timer timer = new Timer();
    private XboxController controller;
    private double waitDuration;
    private double rumbleIntensity;

    public WaitAndVibrateCommand(XboxController controller, double waitDuration, double rumbleIntensity) {
        this.controller = controller;
        this.waitDuration = waitDuration;
        this.rumbleIntensity = rumbleIntensity;
    }

    public WaitAndVibrateCommand(XboxController controller, double waitDuration) {
        this.controller = controller;
        this.waitDuration = waitDuration;
        rumbleIntensity = 0.5;
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
        controller.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        timer.start();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(waitDuration);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        timer.stop();
    }
}
