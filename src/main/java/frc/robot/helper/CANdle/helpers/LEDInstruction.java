package frc.robot.helper.CANdle.helpers;

public class LEDInstruction {

    public LEDColor LEDColor;

    /**
     * Start Index of "Virtualized" LED space
     */
    public int startIdx;
    public int count;

    public LEDInstruction(int r, int g, int b, int w, int startIdx, int count){
        this.LEDColor = new LEDColor(r,g,b,w);

        this.startIdx = startIdx;
        this.count = count;
    }
    public LEDInstruction(LEDColor LEDColor, int startIdx, int count){
        this.LEDColor = LEDColor;

        this.startIdx = startIdx;
        this.count = count;
    }
}
