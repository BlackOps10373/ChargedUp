package org.firstinspires.ftc.teamcode.SharedCode;

import java.util.function.Consumer;

public class curveInterpolator {

    public class dPoint
    {
        public double time; // X
        public double value; // Y
        public dPoint() {}
        public dPoint(double initTime, double initValue)
        {
            time = initTime;
            value = initValue;
        }
    }

    public class weightedPoint extends dPoint
    {
        public double weight;

        weightedPoint(double initX, double initY, double initWeight)
        {
            weight = initWeight;
            time = initX;
            value = initY;
        }
    }



    interface NextPointFunction
    {
        weightedPoint run();
    }

    interface  InterpolationFunction
    {
        double run(weightedPoint prevPoint, weightedPoint nextPoint, double xPosBetween);
    }

    public static double linearInterpolator(dPoint p1, dPoint p2, double xPosBetween)
    {
        double percentP2 = (xPosBetween - p1.time) / (p2.time - p1.time);
        return (percentP2 * p2.value + (1.0 - percentP2) * p1.value);
    }
    public static final InterpolationFunction LlinearInterpolator = (prevPoint, nextPoint, xPosBetween) -> {
        return linearInterpolator(prevPoint, nextPoint, xPosBetween);
    };

    public double curveStretchFactor = 1.0;
    boolean firstTime = true;
    weightedPoint prevPoint;
    weightedPoint nextPoint;

    public double waveOffset = 0.0;
    public NextPointFunction fToGiveNextPoint = () -> {
        return new weightedPoint(0.0, 0.0, 1);
    };
    public InterpolationFunction interpolationFunction;

    curveInterpolator(NextPointFunction initFuncToGiveNextPoint, InterpolationFunction initInterpolationFunc)
    {
        fToGiveNextPoint = initFuncToGiveNextPoint;
        interpolationFunction = initInterpolationFunc;

    }

    double nextFrame(double nextFrameDistance) {
        if (firstTime) {
            nextPoint = fToGiveNextPoint.run();
            if (nextPoint.time == 0.0) {
                prevPoint.value = nextPoint.value;
                //double temp = nextPoint.time; (these two commented lines would be useless because they are for when time is not 0.0 on nextPoint)
                nextPoint = fToGiveNextPoint.run();
                //nextPoint.time += temp; // To make the Time in the function a releative offset
            }

            firstTime = false; // Setting firstTime back to true manualy may not result in expected results because the fToGiveNextPoint is not reset
            return prevPoint.value;
        }
        waveOffset += curveStretchFactor * nextFrameDistance;


        while(waveOffset > nextPoint.time)
        {
            // If waveOffset is not in current domain, get the next one and check again
            prevPoint = nextPoint;
            double temp = nextPoint.time;
            nextPoint = fToGiveNextPoint.run();
            nextPoint.time += temp; // To make the function return a relative time offset
        }
        return interpolationFunction.run(prevPoint, nextPoint, waveOffset);
    }

}
