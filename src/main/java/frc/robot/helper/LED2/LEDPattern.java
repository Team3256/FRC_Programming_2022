package frc.robot.helper.LED2;

public class LEDPattern {
    int startPercentage;
    int endPercentage;
    int length;
    Color[] buffer;
    public LEDPattern(int start, int end){
        startPercentage=start;
        endPercentage=end;
        length=end-start+1;
        buffer = new Color[endPercentage-startPercentage+1];
    }

    public void update(){
        for (int i=0;i<length;i++){
            buffer[i]=new Color(0,0,255);
        }
    }
}
