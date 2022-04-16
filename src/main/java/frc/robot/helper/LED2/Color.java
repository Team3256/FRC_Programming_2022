package frc.robot.helper.LED2;

/**
 * class that holds the state of a RGB color
 * some helper methods implemented
 */
public class Color {
    public int R;
    public int G;
    public int B;
    public Color(int R, int G, int B){
        set(R,G,B);
    }
    public void set(Color color){
        set(color.R, color.G, color.B);
    }
    public void set(int R, int G, int B){
        if (!inColorRange(R)||!inColorRange(G)||!inColorRange(B)) return;
        this.R=R;
        this.G=G;
        this.B=B;
    }
    public boolean inColorRange(int value){
        return !(value < 0 || value > 255);
    }
    public String toString(){
        return "(R:"+R+", G:"+G+", B:"+B+")";
    }
}
