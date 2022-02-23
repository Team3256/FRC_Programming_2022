package frc.robot.helper.CANdle;

public class Color {
    public int r,g,b;

    public Color(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public LEDInstruction toLedInstruction(int startIdx, int count){
        return new LEDInstruction(r, g ,b ,startIdx, count);
    }

    public static final Color off = new Color(0,0,0);

    public static Color fromRGB(int r, int g, int b){
        return new Color(r,g,b);
    }
}
