package frc.robot.helper.CANdle;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.candle.CANdleUpdateCommand;
import frc.robot.subsystems.SwerveDrive;

public class CANdleSystem {

    LEDRaidController ledRaidController;

    public CANdleSystem(CANdle candle, SwerveDrive swerveDrive) {
        this.ledRaidController = new LEDRaidController(candle, swerveDrive);
    }
    public void init(){
        CommandScheduler.getInstance().schedule(new CANdleUpdateCommand(ledRaidController));
    }
}
