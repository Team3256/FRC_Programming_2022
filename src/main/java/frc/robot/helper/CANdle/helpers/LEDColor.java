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

    @Override
    public String toString() {
        return String.format("LEDColor(R:%d, B:%d, G:%d, W:%d",r,b,g,w);
    }

    public static final LEDColor off = new LEDColor(0, 0, 0, 0);

    public static LEDColor fromRGB(int r, int g, int b){
        return new LEDColor(r, g, b, 0);
    }

    @Override
    public boolean equals(Object obj) {
        LEDColor ledColor = (LEDColor) obj;
        return ledColor.r == r && ledColor.g == g && ledColor.b == b && ledColor.w == w;
    }
}
