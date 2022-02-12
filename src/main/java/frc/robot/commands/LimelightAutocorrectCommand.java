package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helper.FileUtil;
import frc.robot.helper.Limelight;
import frc.robot.helper.Polynomial;
import frc.robot.subsystems.SwerveDrive;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

import static frc.robot.Constants.LimelightAutoCorrectConstants.*;

/**
 * Constructor: robotDrive to move forward, initial distance away from goal
 * initial distance will keep track of the robot's actual distance
 * this will be compared to the detected distance to plot error points
 * Purpose: Automatically generate a limelight distance corrector function
 * Runs LimelightAutocorrectStepCommand [PACES] times
 * Make a polynomial of degree [POLYNOMIAL DEGREE] that fits the points of collected data
 * Write polynomial to Polynomial.txt
 */
public class LimelightAutocorrectCommand extends CommandBase {
    private WeightedObservedPoints data;
    private final SwerveDrive robotDrive;
    private int counter;
    private final int initDistance;
    private int curDistance;

    private final PolynomialCurveFitter fitter = PolynomialCurveFitter.create(POLYNOMIAL_DEGREE);

    public LimelightAutocorrectCommand(SwerveDrive robotDrive, int initDistance){
        this.robotDrive = robotDrive;
        this.initDistance = initDistance;
    }

    @Override
    public void initialize() {
        counter = 0;
        data = new WeightedObservedPoints();
        curDistance = initDistance;
    }

    @Override
    public void execute(){
        if (!CommandScheduler.getInstance().isScheduled()) {
            CommandScheduler.getInstance().schedule(new LimelightAutocorrectStepCommand(robotDrive, data, curDistance));
            counter++;
            curDistance-=PACE_SIZE;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Polynomial result = new Polynomial(fitter.fit(data.toList()));
        FileUtil.writeObjectToFile(POLYNOMIAL_FILENAME, result);
        Limelight.readCorrectorFromFile();
    }

    @Override
    public boolean isFinished(){
        return counter >= PACES;
    }
}
