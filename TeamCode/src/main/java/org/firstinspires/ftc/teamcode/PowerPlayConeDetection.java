package org.firstinspires.ftc.teamcode;

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

public class PowerPlayConeDetection extends OpenCvPipeline{
    WebcamName logiCam;
    OpenCvCamera camera;
    Telemetry telemetry;

    //Initializing Matricies, One required for each color and 2 extra for bitwising red, which can be seen below
    Mat mat = new Mat();
    Mat whiteMat = new Mat();
    Mat blackMat = new Mat();
    Mat redMat = new Mat();
    Mat redLMat = new Mat();
    Mat redHMat = new Mat();

    // enum for the three colors
    public enum COLOR {
        WHITE,
        BLACK,
        RED,
    }
    private PowerPlayConeDetection.COLOR color;

    // Defining the rectangle where we want to detect
    static final Rect focusArea = new Rect(
            new Point(90, 130),
            new Point ( 150, 190));

    //This is the percentage that need to be for the color to be officially sensed (0 to 1)
    static double PERCENT_COLOR_THRESHOLD = 0.6;
    public PowerPlayConeDetection(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        //Converts Image to HSV which is easier to sense
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Upper and lower end of each HSV value for color. First is Hue, 0-360/2 for degrees, second is Saturation, 0-255, Third is Value, 0-255
        Scalar lowWhiteRGB = new Scalar(0,0,150);
        Scalar highWhiteRGB = new Scalar(180,50,255);
        // Core.inRange takes the in put, in this case mat, and chech every pixel for if it is in between the two scalers defined above, changing the color to black if it is wrong and white if it is correct, outputing to in this case whiteMat
        Core.inRange(mat, lowWhiteRGB, highWhiteRGB, whiteMat);
        //Creates a submat of the just converted Matrix of the specified size of the rectangle
        Mat whiteSub = whiteMat.submat(focusArea);

        //Repeat for black
        Scalar lowBlackRGB = new Scalar(0,0,0);
        Scalar highBlackRGB = new Scalar(180,255,100);
        Core.inRange(mat, lowBlackRGB, highBlackRGB, blackMat);
        Mat blackSub = blackMat.submat(focusArea);

        //Repeat for Red
        Scalar lowLRedRGB = new Scalar(0,50,20);
        Scalar highLRedRGB = new Scalar(5,255,255);

        Scalar lowHRedRGB = new Scalar(175,50,20);
        Scalar highHRedRGB = new Scalar(180,255,255);

        //Here we do it twice, once for each end of the Hue spectrum, because red begins and ends around 360
        Core.inRange(mat, lowLRedRGB, highLRedRGB, redLMat);
        Core.inRange(mat, lowHRedRGB, highHRedRGB, redHMat);
        //We bitwise the two arrays together, so that it accounts for both colors.
        Core.bitwise_or(redLMat, redHMat, redMat);
        Mat redSub = redMat.submat(focusArea);

        //Takes the sum of the correct pixels in each submat and devides it by the area of the submat, so we can find the percentage
        double whiteValue = Core.sumElems(whiteSub).val[0] / focusArea.area() / 255;
        double blackValue = Core.sumElems(blackSub).val[0] / focusArea.area() / 255;
        double redValue = Core.sumElems(redSub).val[0] / focusArea.area() / 255;

        telemetry.addData("white raw value", whiteValue);
        telemetry.addData("black raw value", blackValue);
        telemetry.addData("red raw value", redValue);

        boolean colorWhite = whiteValue > PERCENT_COLOR_THRESHOLD;
        boolean colorBlack = blackValue > PERCENT_COLOR_THRESHOLD;
        boolean colorRed = redValue > PERCENT_COLOR_THRESHOLD;

        if (colorWhite)
        {
            color = COLOR.WHITE;
            Scalar white = new Scalar(255, 255, 255);
            Imgproc.rectangle(input, focusArea, white);
            telemetry.addData("Object Color", "white");
        }
        else if (colorBlack)
        {
            color = COLOR.BLACK;
            Scalar black = new Scalar(50, 50, 50);
            Imgproc.rectangle(input, focusArea, black);
            telemetry.addData("Object Color", "black");
        }
        else if (colorRed)
        {
            color = COLOR.RED;
            Scalar red = new Scalar(255, 0, 0);
            Imgproc.rectangle(input, focusArea, red);
            telemetry.addData("Object Color", "red");
        }
        else
        {
            Scalar green = new Scalar(0, 255, 0);
            Imgproc.rectangle(input, focusArea, green);
        }

        telemetry.update();

        whiteSub.release();
        blackSub.release();
        redSub.release();

        return input;
    }

    public PowerPlayConeDetection.COLOR getColor()
    {
        return color;
    }
}
