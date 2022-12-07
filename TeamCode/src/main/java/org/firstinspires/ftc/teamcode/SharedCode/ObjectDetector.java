package org.firstinspires.ftc.teamcode.SharedCode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class ObjectDetector extends OpenCvPipeline {
    WebcamName logiCam;
    OpenCvCamera camera;
    Telemetry telemetry;
    Mat mat = new Mat();

    // NEED TO CHANGE THESE VARIABLES AT SOME POINT
    public enum Location {
        LEFT,
        RIGHT,
        MIDDLE,
    }
    private Location location;
    // here
    static final Rect LEFT_ROI = new Rect(
            new Point(50, 60),
            new Point(120, 130));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(160, 40),
            new Point ( 230, 110));
    static final Rect RIGHT_ROI = new Rect(
            new Point(260, 40),
            new Point(320, 110));
    //here
    static double PERCENT_COLOR_THRESHOLD = 0.4;
    public ObjectDetector(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(50,100,50);
        Scalar highHSV = new Scalar(200,255,200);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;

        left.release();
        right.release();
        middle.release();

        telemetry.addData("Left raw value", leftValue);
        telemetry.addData("Right raw value", rightValue);
        telemetry.addData("Middle raw value", middleValue);
        telemetry.addData("Left percentage", Math.round(leftValue * 100));
        telemetry.addData("Right percentage", Math.round(rightValue * 100));
        telemetry.addData("Middle percentage", Math.round(middleValue * 100));

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (stoneMiddle) {
            // it is in the middle
            location = Location.MIDDLE;
            telemetry.addData("Object Location", "middle");
        }
        else if (stoneLeft) {
            // it is on the left
            location = Location.LEFT;
            telemetry.addData("Object Location", "left");
        }
        else {
            // it is on the right
            location = Location.RIGHT;
            telemetry.addData("Object Location", "right");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0,255,0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? colorSkystone:colorStone);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}