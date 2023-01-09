package org.firstinspires.ftc.teamcode.SharedCode;

public class GraphManager {

    public native void PlacePoint(double x, double y);
    public native boolean SaveGraph();
    public native boolean LoadGraph();

    static {
        System.loadLibrary("ftcrobotcontroller");
    }

}
