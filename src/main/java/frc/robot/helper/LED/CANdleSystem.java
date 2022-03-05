package frc.robot.helper.LED;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.candle.CANdleUpdateCommand;
import frc.robot.subsystems.SwerveDrive;

public class CANdleSystem {

    LEDRaidController ledRaidController;

    SwerveDrive swerveDrive;

    public CANdleSystem(SwerveDrive swerveDrive) {
        this.ledRaidController = new LEDRaidController(swerveDrive);
    }

    public void init(){
        CommandScheduler.getInstance().schedule(new CANdleUpdateCommand(ledRaidController));
    }
}
