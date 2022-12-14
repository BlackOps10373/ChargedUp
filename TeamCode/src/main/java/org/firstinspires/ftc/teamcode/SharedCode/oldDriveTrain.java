package org.firstinspires.ftc.teamcode.SharedCode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Arrays;

public class oldDriveTrain {
    // Declarations of all hardware
    Telemetry telemetry;
    DcMotor lw;
    DcMotor rw;
    DcMotor blw;
    DcMotor brw;
    DcMotor frontTwist;
    DcMotor backTwist;
    CRServo treadLeft;
    CRServo treadRight;
    BNO055IMU imu;

    // Target Positions of frontTwist and backTwist
    int frontTargetPosition = 0;
    int backTargetPosition = 0;

    // Variables for Rock Crawler
    int frontTwistDelay = 0;
    int backTwistDelay = 0;
    boolean isFlipping = false;
    boolean firstRun = false;
    boolean forwardFacing = true;
    int blackCount;

    // Variables for Driving
    double lwPower;
    double rwPower;
    double blwPower;
    double brwPower;
    double speedAdjust = .5;

    // Variables for Gyro
    double adjSpeed = 0.03; // Mostly unused, but may still be needed
    double minTurn = 0.01;
    int windowSize = 3;
    double targetDegree = 0.0;
    double resetTargetDegree = 0.0;
    int piecewiseWindow = 179; // 179 to essentially disable it
    double piecewiseSpeed = 0.007517647057771725; // This one is the main speed it uses to get to the target degree
    double piecewiseMinTurn = 0.004;
    double turnSpeed = 5;

    // Lower moveTurnRatio means you turn more when moving (set between 0-1) (1 is 100% turn priority over moving)
    double moveTurnRatio = 0.7;

    double moveSpeed = (7.0 / 10);


    // Drive Train Constructor
    public oldDriveTrain(Telemetry t, HardwareMap hardwareMap) {
        telemetry = t;
        lw = hardwareMap.get(DcMotor.class, "lw");
        rw = hardwareMap.get(DcMotor.class, "rw");
        rw.setDirection(DcMotor.Direction.REVERSE);
        blw = hardwareMap.get(DcMotor.class, "blw");
        brw = hardwareMap.get(DcMotor.class, "brw");
        blw.setDirection(DcMotor.Direction.REVERSE);
        brw.setDirection(DcMotorSimple.Direction.REVERSE);
        lw.setMode(RUN_WITHOUT_ENCODER);
        blw.setMode(RUN_WITHOUT_ENCODER);
        rw.setMode(STOP_AND_RESET_ENCODER);
        rw.setMode(RUN_WITHOUT_ENCODER);
        brw.setMode(RUN_WITHOUT_ENCODER);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) {
            telemetry.addData("isCalibrating", "isCalibrating");
            telemetry.update();
        }
        targetDegree = getHeading();
        resetTargetDegree = targetDegree;

    }



    //Move Function For Auto
    public void move(String variation, int ticCount) {
        int[] ticks = new int[4];
        lw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        switch (variation) {
            case "rotate":
                lw.setTargetPosition(-ticCount);
                blw.setTargetPosition(-ticCount);
                rw.setTargetPosition(ticCount);
                brw.setTargetPosition(ticCount);
                break;
            case "straight":
                lw.setTargetPosition(ticCount);
                blw.setTargetPosition(ticCount);
                rw.setTargetPosition(ticCount);
                brw.setTargetPosition(ticCount);
                break;
            case "diagonalRight":
                blw.setTargetPosition(ticCount);
                rw.setTargetPosition(ticCount);
                break;
            case "diagonalLeft":
                lw.setTargetPosition(ticCount);
                brw.setTargetPosition(ticCount);
                break;
            case "side":
                lw.setTargetPosition(-ticCount);
                blw.setTargetPosition(ticCount);
                rw.setTargetPosition(ticCount);
                brw.setTargetPosition(-ticCount);
                break;
        }
        lw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (lw.isBusy() || rw.isBusy() || brw.isBusy()) {
            //holdPosition();
            //telemetry.addData("rw", rw.getCurrentPosition());
            //telemetry.addData("lw", lw.getCurrentPosition());
            //telemetry.addData("brw", brw.getCurrentPosition());
            //telemetry.addData("blw", blw.getCurrentPosition());
            // telemetry.update();

            ticks[0] = Math.abs(lw.getTargetPosition() - lw.getCurrentPosition());
            ticks[1] = Math.abs(blw.getTargetPosition() - blw.getCurrentPosition());
            ticks[2] = Math.abs(brw.getTargetPosition() - brw.getCurrentPosition());
            ticks[3] = Math.abs(rw.getTargetPosition() - rw.getCurrentPosition());
            Arrays.sort(ticks);

            lwPower = (lw.getTargetPosition() - lw.getCurrentPosition()) * 1.0 / 2;
            blwPower = (blw.getTargetPosition() - blw.getCurrentPosition()) * 1.0 / 2;
            rwPower = (rw.getTargetPosition() - rw.getCurrentPosition()) * 1.0 / 2;
            brwPower = (brw.getTargetPosition() - brw.getCurrentPosition()) * 1.0 / 2;
            telemetry.addData("blw status", blw.getCurrentPosition());
            telemetry.update();

            //gyroStraight();

            if (ticks[3] > 1000) {
                lw.setPower(lwPower / 3);
                blw.setPower(blwPower / 3);
                rw.setPower(rwPower / 3);
                brw.setPower(brwPower / 3);
            } else if (ticks[3] < 500) {
                lw.setPower(lwPower / 8);
                blw.setPower(blwPower / 8);
                rw.setPower(rwPower / 8);
                brw.setPower(brwPower / 8);
            } else {
                lw.setPower(lwPower / 5);
                blw.setPower(blwPower / 5);
                rw.setPower(rwPower / 5);
                brw.setPower(brwPower / 5);
            }
        }
        lw.setPower(0);
        rw.setPower(0);
        blw.setPower(0);
        brw.setPower(0);
    }

    public double degreeCalc(double degree) {
        double returnDegree = degree;
        if (returnDegree < 0) {
            returnDegree = returnDegree + 360;

            return degreeCalc(returnDegree);
        }

        if (returnDegree >= 360) {
            returnDegree = returnDegree - 360;
            return degreeCalc(returnDegree);
        }
        return returnDegree;

    }

    public double degreeCalc180(double degree) {
        double returnDegree = degree;
        if (returnDegree < -180) {
            returnDegree = returnDegree + 360;
            return degreeCalc180(returnDegree);
        } else if (returnDegree >= 180) {
            returnDegree = returnDegree - 360;
            return degreeCalc180(returnDegree);
        }
        return returnDegree;

    }

    public void resetPowers()
    {
        lwPower = 0;
        rwPower = 0;
        blwPower = 0;
        brwPower = 0;
    }

    public double greatest(double a, double b) {
        // returns the greatest of two numbers
        return a > b ? a : b;
        // A warning says this does the same thing as Math.max    Maybe we should use that instead
    }

    public void turnPower(double amount) {

        lwPower += amount;
        rwPower -= amount;
        blwPower += amount;
        brwPower -= amount;

        // Preserves the ratios of each wheel, but changes all magnitudes to be 1 or less
        double a = Math.abs(greatest(greatest(Math.abs(lwPower), Math.abs(rwPower)), greatest(Math.abs(blwPower), Math.abs(brwPower))));
        if (a > 1) {
            lwPower /= a;
            rwPower /= a;
            blwPower /= a;
            brwPower /= a;
        }
    }

    public void gyroStraight() {
        targetDegree = degreeCalc180(targetDegree);

        // function farther from 0 (targetDegree)
        if (degreeCalc(getHeading() - targetDegree) > windowSize + piecewiseWindow && degreeCalc(getHeading() - targetDegree) <= 180) {
            if ((Math.pow(degreeCalc(getHeading() - targetDegree) * adjSpeed, 2)) >= minTurn) {
                turnPower(-(Math.pow(degreeCalc(getHeading() - targetDegree) * adjSpeed, 2)));
            } else {
                turnPower(-minTurn);
            }
            //turnPower(-.5);

        }
        if (degreeCalc(getHeading() - targetDegree) < 360 - windowSize - piecewiseWindow && degreeCalc(getHeading() - targetDegree) > 180) {
            if ((Math.pow((360 - degreeCalc(getHeading() - targetDegree)) * adjSpeed, 2) >= minTurn)) {
                turnPower(Math.pow((360 - degreeCalc(getHeading() - targetDegree)) * adjSpeed, 2));
            } else {
                turnPower(minTurn);
            }
            //turnPower(.5);
        }

        // (piecewise) the one that is closer to 0 degrees (targetDegree)
        if (degreeCalc(getHeading() - targetDegree) > windowSize && degreeCalc(getHeading() - targetDegree) <= piecewiseWindow + windowSize) {
            if ((Math.sqrt(degreeCalc(getHeading() - targetDegree) * piecewiseSpeed)) >= minTurn) {
                turnPower(-(Math.sqrt(degreeCalc(getHeading() - targetDegree) * piecewiseSpeed)));
            } else {
                turnPower(-minTurn);
            }
            //turnPower(-.1);
        }
        if (degreeCalc(getHeading() - targetDegree) < 360 - windowSize && degreeCalc(getHeading() - targetDegree) > 360 - piecewiseWindow - windowSize) {
            if (Math.sqrt((360 - degreeCalc(getHeading() - targetDegree)) * piecewiseSpeed) >= minTurn) {
                turnPower((Math.sqrt((360 - degreeCalc(getHeading() - targetDegree)) * piecewiseSpeed) + piecewiseMinTurn));
            } else {
                turnPower(minTurn);
            }
            //turnPower(.1);
        }
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angles.firstAngle;
    }

    public void move(double YComponent, double XComponent, double Rotate) {
        double driveTurn = -Rotate;
        double XCoordinate = XComponent;
        double YCoordinate = YComponent;

        double gamepadHypot = Range.clip(Math.hypot(XCoordinate, YCoordinate), 0, 1);
        double gamepadDegree = -(Math.toDegrees(Math.atan2(YCoordinate, XCoordinate)) - 90);
        gamepadDegree = degreeCalc180(gamepadDegree);
        //the inverse tangent of opposite/adjacent gives us our gamepad degree
        double robotDegree = getHeading();
        //gives us the angle our robot is at
        double movementDegree = gamepadDegree - robotDegree;

        //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
        double gamepadXControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
        //by finding the adjacent side, we can get our needed x value to power our motors
        double gamepadYControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
        //by finding the opposite side, we can get our needed y value to power our motors
        rwPower = (gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn) * speedAdjust;
        brwPower = (gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn) * speedAdjust;
        lwPower = (gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn) * speedAdjust;
        brwPower = (gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn) * speedAdjust;

        /*
        rw.setPower((gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn) * speedAdjust);
        brw.setPower((gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn) * speedAdjust);
        lw.setPower((gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn) * speedAdjust);
        blw.setPower((gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn) * speedAdjust);
    */
    }

    public void setWheelsToPowers()
    {
        lw.setPower(lwPower);
        rw.setPower(rwPower);
        blw.setPower(blwPower);
        brw.setPower(brwPower);
    }

    public void setEveryMove(double YComponent, double XComponent, double Rotate, double ticks, DcMotor wheel, LinearOpMode opMode, int armUpDown, int armRotation, ObjectGrab objectGrab)
    {
        // If you want more precise control over movement, use the normal move function in your own while loop

        objectGrab.safeArmMovement(armUpDown, armRotation);
        // The distance in not in any established units
        double distanceTraveled = 0;

        int startTicks = wheel.getCurrentPosition();
        while(distanceTraveled < ticks && opMode.opModeIsActive())
        {
            telemetry.addData("ticks", wheel.getCurrentPosition());
            telemetry.update();

            move(YComponent, XComponent, Rotate);
            distanceTraveled = Math.abs(wheel.getCurrentPosition() - startTicks);
            objectGrab.armMovementCheck();
        }
    }
}