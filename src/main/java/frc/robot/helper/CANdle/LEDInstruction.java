package frc.robot.helper.CANdle;

public class LEDInstruction {

    public int r, g, b;

    /**
     * Start Index of "Virtualized" LED space
     */
    public int startIdx;
    public int count;

    public LEDInstruction(int r, int g, int b, int startIdx, int count){
        this.r = r;
        this.g = g;
        this.b = b;

        this.startIdx = startIdx;
        this.count = count;
    }
}
