package frc.robot.commands;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDrive;
import org.apache.commons.math3.analysis.function.Power;

import java.util.logging.Logger;

public class BrownoutWatcher extends CommandBase {
    private static final Logger logger = Logger.getLogger(BrownoutWatcher.class.getCanonicalName());

    PowerDistribution powerDistribution = new PowerDistribution();

    @Override
    public void execute() {
        PowerDistributionFaults faults = powerDistribution.getFaults();
        if (faults.Brownout)
            logger.severe("Brownout Detected!");
        else if (faults.HardwareFault)
            logger.severe("PDP Hardware Fault!");
        else if (faults.CanWarning)
            logger.severe("PDP CAN Warning!");
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
