package frc.robot.helper.CANdle.helpers;

public class LEDInstruction {

    public Color color;

    /**
     * Start Index of "Virtualized" LED space
     */
    public int startIdx;
    public int count;

    public LEDInstruction(int r, int g, int b, int w, int startIdx, int count){
        this.color = new Color(r,g,b,w);

        this.startIdx = startIdx;
        this.count = count;
    }
    public LEDInstruction(Color color, int startIdx, int count){
        this.color = color;

        this.startIdx = startIdx;
        this.count = count;
    }
}
