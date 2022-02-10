package frc.robot.helper;
import java.io.*;
public class Polynomial implements Serializable {
    double[] coefficients;
    int degree;
    public Polynomial(double[] coefficients){
        this.coefficients=coefficients;
        degree = coefficients.length-1;
    }
    public double getOutput(double x){
        double ret = 0;
        for (int i=0;i<coefficients.length;i++){
            ret += coefficients[i]*Math.pow(x,i);
        }
        return ret;
    }
}
