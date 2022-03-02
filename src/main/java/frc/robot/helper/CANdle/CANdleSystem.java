package frc.robot.helper.CANdle;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.candle.CANdleUpdateCommand;
import frc.robot.subsystems.SwerveDrive;

import static frc.robot.Constants.IDConstants.CANDLE_ID;

public class CANdleSystem {

    CANdle candle;
    LEDRaidController ledRaidController;

    SwerveDrive swerveDrive;

    public CANdleSystem(SwerveDrive swerveDrive) {
        candle = new CANdle(CANDLE_ID);
        this.swerveDrive = swerveDrive;
        this.ledRaidController = new LEDRaidController(candle, swerveDrive);
    }

    public void init(){
        CommandScheduler.getInstance().schedule(new CANdleUpdateCommand(ledRaidController));
    }
}
