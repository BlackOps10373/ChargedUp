package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SharedCode.GraphManager;
import org.firstinspires.ftc.teamcode.SharedCode.curveInterpolator;

import org.firstinspires.ftc.teamcode.SharedCode.DriveTrain;

@TeleOp(name = "RedSideTeleOp", group = "TeleOp")
public class RedSideTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, DriveTrain.Alliance.Red);
        AILS ails = new AILS(telemetry, hardwareMap);

        boolean dpadUp = false;
        boolean dpadDown = false;
        boolean dpadLeft = false;
        boolean dpadRight = false;

        double InitialYComponent = 0;
        double InitialXComponent = 0;

        int currentTargetX = 0;
        int currentTargetY = 0;

        int leftTics = 0;
        int rightTics = 0;
        int leftOffset = 0;
        int rightOffset = 0;

        //while (!driveTrain.imu.isGyroCalibrated()) {
            telemetry.addLine("Robot Is Initializing: DO NOT START");
            telemetry.update();
        //}
        telemetry.addLine("Initialization Complete: Ready To Start");
        telemetry.update();

        waitForStart();
        //leftStick = new Vector2D((double) gamepad1.left_stick_x, (double) -gamepad1.left_stick_y);
        //rightStick = new Vector2D((double) gamepad1.right_stick_x, (double) gamepad1.right_stick_y);
        while(opModeIsActive() && !isStopRequested()){

            driveTrain.updatePosition();

            telemetry.addData("Heading", driveTrain.degreeCalc180(Math.toDegrees(driveTrain.getHeading())));
            telemetry.addData("LeftEncoder", driveTrain.leftEncoderMotor.getCurrentPosition());
            telemetry.addData("RightEncoder", driveTrain.rightEncoderMotor.getCurrentPosition());
            telemetry.addData("CenterEncoder", driveTrain.centerEncoderMotor.getCurrentPosition());
            telemetry.addData("right Chain", ails.zipChainRight.getCurrentPosition());
            telemetry.addData("left Chain", ails.zipChainLeft.getCurrentPosition());


            telemetry.addData("Y value", driveTrain.getY());
            telemetry.addData("X value", driveTrain.getX());
            telemetry.addData("Position", driveTrain.getPosition());
            telemetry.addData("Rotation", driveTrain.getHeading());
            telemetry.update();

            if (gamepad1.a)
            {
                if(driveTrain.updateWithViewforia()) {
                    gamepad1.rumble(400);
                }
            }

            if (gamepad2.b)
                ails.grabber.setPosition(.6);
            if (gamepad2.a)
                ails.grabber.setPosition(.5);

            if (gamepad1.dpad_up && !dpadUp) {
                driveTrain.pathPoints[driveTrain.finalArrayPointInit] = new Vector2D(currentTargetX, currentTargetY + 24);
                currentTargetY += 24;
                driveTrain.finalArrayPointInit++;
                dpadUp = true;
            } else if (!gamepad1.dpad_up)
                dpadUp = false;

            if (gamepad1.dpad_down && !dpadDown) {
                driveTrain.pathPoints[driveTrain.finalArrayPointInit] = new Vector2D(currentTargetX, currentTargetY - 24);
                currentTargetY -= 24;
                driveTrain.finalArrayPointInit++;
                dpadDown = true;
            } else if (!gamepad1.dpad_down)
                dpadDown = false;

            if (gamepad1.dpad_right && !dpadRight) {
                driveTrain.pathPoints[driveTrain.finalArrayPointInit] = new Vector2D(currentTargetX + 24, currentTargetY);
                currentTargetX += 24;
                driveTrain.finalArrayPointInit++;
                dpadRight = true;
            } else if (!gamepad1.dpad_right)
                dpadRight = false;

            if (gamepad1.dpad_left && !dpadLeft) {
                driveTrain.pathPoints[driveTrain.finalArrayPointInit] = new Vector2D(currentTargetX - 24, currentTargetY);
                currentTargetX -= 24;
                driveTrain.finalArrayPointInit++;
                dpadLeft = true;
            } else if (!gamepad1.dpad_left)
                dpadLeft = false;


            if(driveTrain.pathPoints[driveTrain.currentArrayPointWorking] == null){
                InitialYComponent = gamepad1.left_stick_y;
                InitialXComponent = gamepad1.left_stick_x;
            } else {
                if (driveTrain.pathPoints[driveTrain.currentArrayPointWorking].getComponent(0) - driveTrain.getX() > 0)
                    InitialXComponent = 0.5;
                else if (driveTrain.pathPoints[driveTrain.currentArrayPointWorking].getComponent(0) - driveTrain.getX() < 0)
                    InitialXComponent = -0.5;
                else
                    InitialXComponent = 0;

                if (driveTrain.pathPoints[driveTrain.currentArrayPointWorking].getComponent(1) - driveTrain.getY() > 0)
                    InitialYComponent = 0.5;
                else if (driveTrain.pathPoints[driveTrain.currentArrayPointWorking].getComponent(1) - driveTrain.getY() < 0)
                    InitialYComponent = -0.5;
                else
                    InitialYComponent = 0;

                if(Math.abs(driveTrain.pathPoints[driveTrain.currentArrayPointWorking].getComponent(0) - driveTrain.getX()) < 1
                        && Math.abs(driveTrain.pathPoints[driveTrain.currentArrayPointWorking].getComponent(1) - driveTrain.getY()) < 1)
                    driveTrain.currentArrayPointWorking++;
            }
            driveTrain.move(InitialYComponent, InitialXComponent, gamepad1.right_stick_x);
            //leftTics += gamepad1.right_stick_y;
            //rightTics += gamepad1.right_stick_y;
            //ails.zip(leftTics + leftOffset, rightTics + rightOffset);


            if ((ails.zipChainRight.getCurrentPosition() < 2720 && ails.zipChainLeft.getCurrentPosition() < 2720)) {
                ails.zipChainLeft.setPower(gamepad2.right_stick_y * .75);
                ails.zipChainRight.setPower(gamepad2.right_stick_y * .75);
            } else {
                ails.zipChainLeft.setPower(0);
                ails.zipChainRight.setPower(0);
            }



            /*driveTrain.lw.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            driveTrain.rw.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            driveTrain.blw.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
            driveTrain.brw.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);*/
            //driveTrain.move(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        driveTrain.endViewforia();
    }
}