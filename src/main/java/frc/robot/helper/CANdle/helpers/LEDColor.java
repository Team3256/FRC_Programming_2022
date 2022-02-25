package frc.robot.helper.CANdle.helpers;

public class LEDColor {
    public int r,g,b,w;

    public LEDColor(int r, int g, int b, int w) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.w = w;
    }

    public LEDInstruction toLedInstruction(int startIdx, int count){
        return new LEDInstruction(this, startIdx, count);
    }

    public static final LEDColor off = new LEDColor(0, 0, 0, 0);

    public static LEDColor fromRGB(int r, int g, int b){
        return new LEDColor(r, g, b, 0);
    }
}
