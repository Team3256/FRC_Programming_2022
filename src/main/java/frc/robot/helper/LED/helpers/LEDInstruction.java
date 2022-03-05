package frc.robot.helper.LED.helpers;

public class LEDInstruction {

    public LEDColor ledColor;

    /**
     * Start Index of "Virtualized" LED space
     */
    public int startIdx;
    public int count;

    public LEDInstruction(int r, int g, int b, int w, int startIdx, int count){
        this.ledColor = new LEDColor(r,g,b,w);

        this.startIdx = startIdx;
        this.count = count;
    }
    public LEDInstruction(LEDColor ledColor, int startIdx, int count){
        this.ledColor = ledColor;

        this.startIdx = startIdx;
        this.count = count;
    }

    @Override
    public String toString() {
        return String.format("LED Instruction( Color: %s, Start Index: %d, Count: %d)",
                ledColor.toString(), startIdx, count);
    }

    @Override
    public boolean equals(Object obj) {
        LEDInstruction ledInstruction = (LEDInstruction) obj;
        return ledColor.equals(ledInstruction.ledColor) && ledInstruction.startIdx == startIdx && ledInstruction.count == count;
    }
}
