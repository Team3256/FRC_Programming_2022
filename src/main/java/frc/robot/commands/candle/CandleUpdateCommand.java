package frc.robot.commands.candle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.CANdle.CANdleSystem;
import frc.robot.helper.CANdle.LEDRaidController;

public class CandleUpdateCommand extends CommandBase {

    LEDRaidController raidController;

    public CandleUpdateCommand(LEDRaidController raidController){
            this.raidController = raidController;
    }

    @Override
    public void execute() {
        raidController.runUpdates();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
