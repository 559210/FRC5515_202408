package frc.robot.utils;

public class CoordinaryFilter {
    private double weight = 0;
    private double value;
    private boolean inited = false;

    // weight is a number between 0~1
    public CoordinaryFilter(double w) {
        if (w < 0 || w > 1) {
            weight = 0;
        }
        else {
            weight = w;
        }
        
    }

    public double calc(double v) {
        if (inited) {
            double ret = value * weight + v * (1 - weight);
            value = v;
            return ret;
        }
        else {
            inited = true;
            value = v;
            return v;
        }
    }

    // int heartbeat = 0;
    // public double calc(double v) {
    //     heartbeat++;
    //     // int freqScale = 20;
    //     // if (heartbeat % freqScale == 0) {
    //     //     value = v * (1 - Math.exp(-))
    //     // }
    //     // else {
    //     //     value = v;
    //     // }
    //     // return value;
    //     return v * (1 - Math.exp(-5 * heartbeat * 0.02));
    // }
}
