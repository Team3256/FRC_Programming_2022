package frc.robot.helper.LED2;

/**
 * class that holds the state of a RGB color
 */
public class Color {
    public int R;
    public int G;
    public int B;
    public Color(int R, int G, int B){
        set(R,G,B);
    }
    public void set(int R, int G, int B){
        this.R=R;
        this.G=G;
        this.B=B;
    }
    public String toString(){
        return "(R:"+R+", G:"+G+", B:"+B+")";
    }
}
