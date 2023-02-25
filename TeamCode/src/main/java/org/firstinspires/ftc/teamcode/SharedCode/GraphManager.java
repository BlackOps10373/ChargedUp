package org.firstinspires.ftc.teamcode.SharedCode;

public class GraphManager {

    public native void PlacePoint(double x, double y, double time);
    public native void getPoint(double time);
    public native double getXOfPoint();
    public native double getYOfPoint();
    //public native boolean SaveGraph();
    //public native boolean LoadGraph();

    static {
        System.loadLibrary("ftcrobotcontroller");
    }

}
