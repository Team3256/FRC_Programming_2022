package frc.robot.commands.candle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.CANdle.LEDRaidController;

import static frc.robot.Constants.CYCLES_PER_CANDLE_UPDATE;

public class CANdleUpdateCommand extends CommandBase {

    LEDRaidController raidController;
    int counter;

    public CANdleUpdateCommand(LEDRaidController raidController){
            this.raidController = raidController;
    }

    @Override
    public void initialize() {
        counter = 0;
    }

    @Override
    public void execute() {
        raidController.update();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
