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

public class DriveTrain {
    // Declarations of all hardware
    Telemetry telemetry;
    HardwareMap hardwareMap;
    public DcMotor lw;
    public DcMotor rw;
    public DcMotor blw;
    public DcMotor brw;
    DcMotor frontTwist;
    DcMotor backTwist;
    CRServo treadLeft;
    CRServo treadRight;
    BNO055IMU imu;


    // Variables for Driving
    double lwPower;
    double rwPower;
    double blwPower;
    double brwPower;
    double speedAdjust = .5;

    // Lower moveTurnRatio means you turn more when moving (set between 0-1) (1 is 100% turn priority over moving)
    double moveTurnRatio = 0.7;

    double moveSpeed = (7.0 / 10);

    // Drive Train Constructor
    public DriveTrain(Telemetry t,HardwareMap hM) {
        telemetry = t;
        hardwareMap = hM;

        /*imu = hardwareMap.get(BNO055IMU.class, "imu");
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
         */
    }

    public void initMotors()
    {
        // Default settings
        lw = hardwareMap.get(DcMotor.class, "lw");
        rw = hardwareMap.get(DcMotor.class, "rw");
        blw = hardwareMap.get(DcMotor.class, "blw");
        brw = hardwareMap.get(DcMotor.class, "brw");

        rw.setDirection(DcMotor.Direction.REVERSE);
        brw.setDirection(DcMotorSimple.Direction.REVERSE);

        lw.setMode(STOP_AND_RESET_ENCODER);
        rw.setMode(STOP_AND_RESET_ENCODER);
        blw.setMode(STOP_AND_RESET_ENCODER);
        brw.setMode(STOP_AND_RESET_ENCODER);
        lw.setMode(RUN_WITHOUT_ENCODER);
        blw.setMode(RUN_WITHOUT_ENCODER);
        rw.setMode(RUN_WITHOUT_ENCODER);
        brw.setMode(RUN_WITHOUT_ENCODER);
    }
    public void initMotors(DcMotor.RunMode lwMode, DcMotorSimple.Direction lwDirection,
    DcMotor.RunMode rwMode, DcMotorSimple.Direction rwDirection,
    DcMotor.RunMode blwMode, DcMotorSimple.Direction blwDirection,
    DcMotor.RunMode brwMode, DcMotorSimple.Direction brwDirection)
    {
        // Overload for specific settings
        lw = hardwareMap.get(DcMotor.class, "lw");
        rw = hardwareMap.get(DcMotor.class, "rw");
        blw = hardwareMap.get(DcMotor.class, "blw");
        brw = hardwareMap.get(DcMotor.class, "brw");

        lw.setMode(STOP_AND_RESET_ENCODER);
        rw.setMode(STOP_AND_RESET_ENCODER);
        blw.setMode(STOP_AND_RESET_ENCODER);
        brw.setMode(STOP_AND_RESET_ENCODER);
        lw.setMode(lwMode);
        rw.setMode(rwMode);
        blw.setMode(blwMode);
        brw.setMode(brwMode);

        lw.setDirection(lwDirection);
        rw.setDirection(rwDirection);
        brw.setDirection(brwDirection);
    }


    //Move Function For Auto
    public void moveCardinals(String variation, int ticCount) {
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

    void resetPowers()
    {
        lwPower = 0;
        rwPower = 0;
        blwPower = 0;
        brwPower = 0;
    }

    public void turnPower(double amount) {

        lwPower += amount;
        rwPower -= amount;
        blwPower += amount;
        brwPower -= amount;

        // Preserves the ratios of each wheel, but changes all magnitudes to be 1 or less
        double a = Math.abs(Math.max(Math.max(Math.abs(lwPower), Math.abs(rwPower)), Math.max(Math.abs(blwPower), Math.abs(brwPower))));
        if (a > 1) {
            lwPower /= a;
            rwPower /= a;
            blwPower /= a;
            brwPower /= a;
        }
    }

    public void move(double XComponent, double YComponent, double Rotate) {
        double driveTurn = -Rotate;
        double XCoordinate = XComponent;
        double YCoordinate = YComponent;

        /*
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

        rw.setPower((gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn) * speedAdjust);
        brw.setPower((gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn) * speedAdjust);
        lw.setPower((gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn) * speedAdjust);
        blw.setPower((gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn) * speedAdjust);
        */

        rw.setPower(YCoordinate - XCoordinate - driveTurn);
        lw.setPower(YCoordinate + XCoordinate + driveTurn);
        blw.setPower(YCoordinate - XCoordinate - driveTurn);
        brw.setPower(YCoordinate + XCoordinate + driveTurn);
    }
}