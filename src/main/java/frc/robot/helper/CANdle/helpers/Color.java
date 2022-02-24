package frc.robot.helper.CANdle.helpers;

public class Color {
    public int r,g,b,w;

    public Color(int r, int g, int b, int w) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.w = w;
    }

    public LEDInstruction toLedInstruction(int startIdx, int count){
        return new LEDInstruction(this, startIdx, count);
    }

    public static final Color off = new Color(0, 0, 0, 0);

    public static Color fromRGB(int r, int g, int b){
        return new Color(r, g, b, 0);
    }
}
