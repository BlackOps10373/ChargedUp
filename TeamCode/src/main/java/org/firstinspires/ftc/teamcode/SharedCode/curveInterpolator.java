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





    Consumer<Integer> test = p -> {
        p += 2;
        return;
    };

    interface NextPointFunction
    {
        weightedPoint run();
    }

    interface  InterpolationFunction
    {
        double run(weightedPoint prevPoint, weightedPoint nextPoint, double xPosBetween);
    }

    public double linearInterpolator(dPoint p1, dPoint p2, double xPosBetween)
    {
        double percentP2 = (xPosBetween - p1.time) / (p2.time - p1.time);
        return (percentP2 * p2.value + (1.0 - percentP2) * p1.value);
    }


    public double curveStretchFactor = 1.0;
    boolean firstTime = true;
    weightedPoint prevPoint;
    weightedPoint nextPoint;

    public double waveOffset = 0.0;
    public NextPointFunction fToGiveNextPoint;
    public InterpolationFunction interpolationFunction;

    curveInterpolator(NextPointFunction initFuncToGiveNextPoint, InterpolationFunction initInterpolationFunc)
    {
        fToGiveNextPoint = initFuncToGiveNextPoint;
        interpolationFunction = initInterpolationFunc;

    }

}
