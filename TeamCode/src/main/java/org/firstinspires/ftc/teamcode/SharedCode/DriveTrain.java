package org.firstinspires.ftc.teamcode.SharedCode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

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
import org.firstinspires.ftc.teamcode.Vector2D;

import java.util.Arrays;

public class DriveTrain {

    //DECLARATIONS OF ALL HARDWARE
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public DcMotor lw, rw, blw, brw;
    public DcMotor leftEncoderMotor, rightEncoderMotor, centerEncoderMotor;
    public BNO055IMU imu;

    // DECLARATIONS OF ALL VARIABLES
    double lwPower;
    double rwPower;
    double blwPower;
    double brwPower;
    double speedAdjust = .5;
    // Lower moveTurnRatio means you turn more when moving (set between 0-1) (1 is 100% turn priority over moving)
    double moveTurnRatio = 0.7;
    double moveSpeed = (7.0 / 10);

    Orientation angles;
    private final double ENCODER_TICS_PER_INCH = (8192 / ((35/25.4) * Math.PI)); //in inches
    private final double RADIUS = 6; //in inches
    private final double CENTER_ENCODER_RADIUS = 6;

    public static final double oneRotationTicks = 8000;
    public static final double wheelRadius = 0.0175; // in meters
    public static final double wheelDistanceApart = 0.3; // in meters

    private int currentLeftEncoder = 0;
    private int currentRightEncoder = 0;
    private int currentCenterEncoder = 0;

    private int previousLeftEncoder = 0;
    private int previousRightEncoder = 0;
    private int previousCenterEncoder = 0;

    private int deltaLeftEncoder = 0;
    private int deltaRightEncoder = 0;
    private int deltaCenterEncoder = 0;

    private double heading = 0;
    private double deltaHeading = 0;

    private double deltax = 0;
    private double deltay = 0;

    public double fieldDeltaX = 0;
    public double fieldDeltaY = 0;

    Vector2D robotCentricDelta;
    Vector2D fieldCentricDelta;
    Vector2D position = new Vector2D(0,0);
    private double x = 0;
    private double y = 0;
    private double theta = 0;


    // CONSTRUCTOR
    public DriveTrain(Telemetry t,HardwareMap hM) {
        telemetry = t;
        hardwareMap = hM;

        initMotors();
        initIMU();
        //targetDegree = getHeading();
        //resetTargetDegree = targetDegree;
    }

    public void headlessDrive(double yInput, double xInput, double rotateInput){
        double initY = yInput;
        double initX = xInput;
        double initRotate = rotateInput;


    }

    public void updatePosition(){
        updateEncoders();

        deltaLeftEncoder = currentLeftEncoder - previousLeftEncoder;
        deltaRightEncoder = currentRightEncoder - previousRightEncoder;
        deltaCenterEncoder = currentCenterEncoder - previousCenterEncoder;

        previousLeftEncoder = currentLeftEncoder;
        previousRightEncoder = currentRightEncoder;
        previousCenterEncoder = currentCenterEncoder;

        deltaHeading = (deltaRightEncoder - deltaLeftEncoder) / (2.0 * RADIUS * ENCODER_TICS_PER_INCH);
        heading = (currentRightEncoder - currentLeftEncoder) / (2.0 * RADIUS * ENCODER_TICS_PER_INCH);

        if(deltaHeading == 0){ //have to do it like this because java doesn't do l'Hopital's rule
            deltax = deltaCenterEncoder;
            deltay = (deltaLeftEncoder + deltaRightEncoder)/2;
        }else{
            double turnRadius = RADIUS * ENCODER_TICS_PER_INCH * (deltaLeftEncoder + deltaRightEncoder) / (deltaRightEncoder - deltaLeftEncoder);
            double strafeRadius = deltaCenterEncoder / deltaHeading - CENTER_ENCODER_RADIUS * ENCODER_TICS_PER_INCH;

            deltax = turnRadius*(Math.cos(deltaHeading) - 1) + strafeRadius*Math.sin(deltaHeading);
            deltay = turnRadius*Math.sin(deltaHeading) + strafeRadius*(1 - Math.cos(deltaHeading));
            x += encoderToInch(deltax);
            y += encoderToInch(deltay);
        }
        encoderToFieldConversion(deltax, deltay);
        fieldCentricDelta = new Vector2D(encoderToInch(fieldDeltaX), encoderToInch(fieldDeltaY));
        robotCentricDelta = new Vector2D(encoderToInch(deltax), encoderToInch(deltay));

        //fieldCentricDelta = new Vector2D(encoderToInch(deltay), encoderToInch(-deltax));
        //fieldCentricDelta.rotate(heading);
        position.add(fieldCentricDelta);
    }

    public void updateEncoders(){
        currentLeftEncoder = -leftEncoderMotor.getCurrentPosition();
        currentRightEncoder = rightEncoderMotor.getCurrentPosition();
        currentCenterEncoder = -centerEncoderMotor.getCurrentPosition();
    }

    public double encoderToInch(double encoder) {
        return encoder/ENCODER_TICS_PER_INCH;
    }

    public void encoderToFieldConversion(double robotDeltaX, double robotDeltaY){
        fieldDeltaX = robotDeltaX*Math.cos(getHeading()) + robotDeltaY*Math.cos(getHeading() + Math.PI/2);
        fieldDeltaY = robotDeltaX*Math.sin(getHeading()) + robotDeltaY*Math.sin(getHeading() + Math.PI/2);
    }

    public double getHeading(){
        return heading;
    }
    public Vector2D getPosition(){
        return position;
    }
    /*public void updateControllerValues() {
        leftStick.setComponents(new double[] {gamepad1.left_stick_x, -gamepad1.left_stick_y});
        rightStick.setComponents(new double[] {gamepad1.right_stick_x, -gamepad1.right_stick_y});
        Agobot.drivetrain.updatePosition();
        leftStick.rotate(Math.toRadians(driverHeading - Agobot.drivetrain.getHeading()));
        double leftx = leftStick.getComponent(0);
        double lefty = leftStick.getComponent(1);
        double scalar = Math.max(Math.abs(lefty-leftx), Math.abs(lefty+leftx)); //scalar and magnitude scale the motor powers based on distance from joystick origin
        double magnitude = Math.sqrt(Math.pow(lefty, 2) + Math.pow(leftx, 2));

        motorSpeeds = new Vector2D((lefty+leftx)*magnitude/scalar, (lefty-leftx)*magnitude/scalar);
    }
    public void rotate(double radians) {

        theta = (theta + radians) % (2 * Math.PI);
        genComp();
    }
    public void genComp() {
        components = new double[2];

        components[0] = R * Math.cos(theta);
        components[1] = R * Math.sin(theta);
    }
*/

    public void initMotors()
    {
        // Default settings
        lw = hardwareMap.get(DcMotor.class, "lw");
        rw = hardwareMap.get(DcMotor.class, "rw");
        blw = hardwareMap.get(DcMotor.class, "blw");
        brw = hardwareMap.get(DcMotor.class, "brw");

        leftEncoderMotor = hardwareMap.get(DcMotor.class, "lw");
        rightEncoderMotor = hardwareMap.get(DcMotor.class, "blw");
        centerEncoderMotor = hardwareMap.get(DcMotor.class, "rw");

        leftEncoderMotor.setDirection(FORWARD);
        rightEncoderMotor.setDirection(FORWARD);
        centerEncoderMotor.setDirection(REVERSE);

        leftEncoderMotor.setMode(STOP_AND_RESET_ENCODER);
        rightEncoderMotor.setMode(STOP_AND_RESET_ENCODER);
        centerEncoderMotor.setMode(STOP_AND_RESET_ENCODER);
        leftEncoderMotor.setMode(RUN_WITHOUT_ENCODER);
        rightEncoderMotor.setMode(RUN_WITHOUT_ENCODER);
        centerEncoderMotor.setMode(RUN_WITHOUT_ENCODER);

        lw.setDirection(FORWARD);
        rw.setDirection(REVERSE);
        blw.setDirection(FORWARD);
        brw.setDirection(REVERSE);

        lw.setMode(RUN_WITHOUT_ENCODER);
        rw.setMode(RUN_WITHOUT_ENCODER);
        blw.setMode(RUN_WITHOUT_ENCODER);
        brw.setMode(RUN_WITHOUT_ENCODER);

        lw.setZeroPowerBehavior(BRAKE);
        rw.setZeroPowerBehavior(BRAKE);
        blw.setZeroPowerBehavior(BRAKE);
        brw.setZeroPowerBehavior(BRAKE);
    }

    public void initIMU(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
    }


    /*public void resetTicks() {
        resetLeftTicks();
        resetCenterTicks();
        resetRightTicks();
    }
    public void resetLeftTicks() {
        leftEncoderPos = leftEncoderMotor.getCurrentPosition();
    }
    public int getLeftTicks() {
        return -(leftEncoderMotor.getCurrentPosition() - leftEncoderPos);
    }
    public void resetRightTicks() {
        rightEncoderPos = rightEncoderMotor.getCurrentPosition();
    }
    public int getRightTicks() {
        return rightEncoderMotor.getCurrentPosition() - rightEncoderPos;
    }
    public void resetCenterTicks() {
        centerEncoderPos = centerEncoderMotor.getCurrentPosition();
    }
    public int getCenterTicks() {
        return -(centerEncoderMotor.getCurrentPosition() - centerEncoderPos);
    }*/

   /* public void updatePosition() {
        deltaLeftDistance = (getLeftTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaRightDistance = (getRightTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaCenterDistance = (getCenterTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
        y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
        theta  += (deltaLeftDistance - deltaRightDistance) / wheelDistanceApart;
        resetTicks();
    }*/
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getTheta() {
        return theta;
    }
    public void setX(double _x) {
        x = _x;
    }
    public void setY(double _y) {
        y = _y;
    }
    public void setTheta(double _theta) {
        theta = _theta;
    }
    public double angle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
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

    public void move(double YComponent, double XComponent, double Rotate) {
        double driveTurn = Rotate;
        double XCoordinate = XComponent;
        double YCoordinate = -YComponent;

        double gamepadHypot = Range.clip(Math.hypot(XCoordinate, YCoordinate), 0, 1);
        double gamepadDegree = -(Math.toDegrees(Math.atan2(YCoordinate, XCoordinate)) - 90);
        gamepadDegree = degreeCalc180(gamepadDegree);
        //the inverse tangent of opposite/adjacent gives us our gamepad degree
        double robotDegree = (degreeCalc180(-getHeading()*180));
        //gives us the angle our robot is at
        double movementDegree = gamepadDegree - robotDegree;

        //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
        double gamepadXControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
        //by finding the adjacent side, we can get our needed x value to power our motors
        double gamepadYControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
        //by finding the opposite side, we can get our needed y value to power our motors
        //rwPower = (gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn) * speedAdjust;
        //brwPower = (gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn) * speedAdjust;
        //lwPower = (gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn) * speedAdjust;
        //brwPower = (gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn) * speedAdjust;
        double XComponentPower = gamepadXControl / 1.5;
        double YComponentPower = gamepadYControl / 1.5;
        double RotateComponentPower = driveTurn / 1.5;
        rw.setPower((YComponentPower - XComponentPower - RotateComponentPower));
        brw.setPower((YComponentPower + XComponentPower - RotateComponentPower));
        lw.setPower((YComponentPower + XComponentPower + RotateComponentPower));
        blw.setPower((YComponentPower - XComponentPower + RotateComponentPower));
    }

    public void goToPosition(double YComponent, double XComponent, double speed, double preferredAngle) {
        double movementspeed = speed;
        double XCoordinate = XComponent;
        double YCoordinate = -YComponent;

        double distanceToTarget = Range.clip(Math.hypot(XCoordinate, YCoordinate), 0, 1);
        double absoluteAngleToTarget = -(Math.toDegrees(Math.atan2(YCoordinate, XCoordinate)) - 90);
        absoluteAngleToTarget = degreeCalc180(absoluteAngleToTarget);
        //the inverse tangent of opposite/adjacent gives us our gamepad degree
        double robotDegree = getHeading();
        //gives us the angle our robot is at
        double relativeAngleToPoint = absoluteAngleToTarget - robotDegree;

        //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
        double gamepadXControl = Math.sin(Math.toRadians(relativeAngleToPoint)) * distanceToTarget;
        //by finding the adjacent side, we can get our needed x value to power our motors
        double gamepadYControl = Math.cos(Math.toRadians(relativeAngleToPoint)) * distanceToTarget;
        //by finding the opposite side, we can get our needed y value to power our motors
        //rwPower = (gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn) * speedAdjust;
        //brwPower = (gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn) * speedAdjust;
        //lwPower = (gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn) * speedAdjust;
        //brwPower = (gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn) * speedAdjust;
        double XComponentPower = gamepadXControl / (Math.abs(gamepadYControl) + Math.abs(gamepadXControl));
        double YComponentPower = gamepadYControl / (Math.abs(gamepadYControl) + Math.abs(gamepadXControl));
        double relativeTurnAngle = relativeAngleToPoint - 180 + preferredAngle;
    }

    public void rawMove(double XComponent, double YComponent, double Rotate)
    {
        double driveTurn = Rotate;
        double XCoordinate = XComponent;
        double YCoordinate = -YComponent;

        lw.setPower(YCoordinate + XCoordinate + driveTurn);
        rw.setPower(YCoordinate - XCoordinate - driveTurn);
        blw.setPower(YCoordinate - XCoordinate + driveTurn);
        brw.setPower(YCoordinate + XCoordinate - driveTurn);
    }

   public double getIMUHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angles.firstAngle;
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

    /*public void move(double XComponent, double YComponent, double Rotate) {
        double driveTurn = Rotate;
        double XCoordinate = XComponent;
        double YCoordinate = -YComponent; // The stick outputs up as negative, so this changes it to positive

        double newYCoord = YCoordinate;
        double newXCoord = XCoordinate;


        double gamepadHypot = 0.0; // magnitude of the vector
        double gamepadAngleRad = 0.0; // angle (in radians) of the vector
        if(Math.abs(XCoordinate) == 1.0 || Math.abs(YCoordinate) == 1.0)
        {
            // for undoing stick snapping the best I can:
            gamepadHypot = 1.0;
            if(Math.abs(XCoordinate) == 1.0)
            {
                // XCoordinate is likely not actually 1.0, so we use the other cord to get the degree
                gamepadAngleRad = Math.asin(YCoordinate);
            }
            if(Math.abs(YCoordinate) == 1.0)
            {
                // YCoordinate is likely not actually 1.0, so we use the other cord to get the degree
                gamepadAngleRad = 90 + Math.asin(XCoordinate);
            }
        }
        else
        {
            gamepadHypot = Range.clip(Math.hypot(XCoordinate, YCoordinate), 0, 1);
            gamepadAngleRad = Math.atan2(YCoordinate, XCoordinate);
        }
        gamepadAngleRad = getDegreeInRange_PosNeg180(gamepadAngleRad);
        double robotDegree = getHeading();
        double movementDegree = gamepadAngleRad - robotDegree;

        // get the new coordinates for the vector after the angle is adjusted
        double newYCoord = Math.sin(movementDegree) * gamepadHypot;
        double newXCoord = Math.cos(movementDegree) * gamepadHypot;


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


        //lw.setPower(newYCoord + newXCoord + driveTurn);
        //rw.setPower(newYCoord - newXCoord - driveTurn);
        //blw.setPower(newYCoord - newXCoord + driveTurn);
        //brw.setPower(newYCoord + newXCoord - driveTurn);
    }

    double getDegreeInRange_PosNeg180(double degree)
    {
        double returnDegree = degree;

        while (returnDegree < -180 || returnDegree >= 180) {
            if (returnDegree < -180) {
                returnDegree = returnDegree + 360;
            } else if (returnDegree >= 180) {
                returnDegree = returnDegree - 360;
            }
        }
        return returnDegree;
    }*/
}