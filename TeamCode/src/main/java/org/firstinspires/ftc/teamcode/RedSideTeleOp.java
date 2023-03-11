package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SharedCode.GraphManager;
import org.firstinspires.ftc.teamcode.SharedCode.curveInterpolator;

import org.firstinspires.ftc.teamcode.SharedCode.DriveTrain;
import org.firstinspires.ftc.teamcode.SharedCode.Button;

@TeleOp(name = "RedSideTeleOp", group = "TeleOp")
public class RedSideTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, DriveTrain.Alliance.Red);
        AILS ails = new AILS(telemetry, hardwareMap);
        GraphManager graphManager = new GraphManager();

        Button up = new Button();
        Button down = new Button();
        Button right = new Button();
        Button left = new Button();
        Button bX = new Button();

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


            if (up.OnButtonDown(gamepad1.dpad_up))
            {
                double[] pos = null;
                if (time > graphManager.getLastPointTime()) {
                    pos = driveTrain.getFieldTileCenter().getComponents();
                    graphManager.PlacePoint(pos[0], pos[1], time);
                }
                else
                {
                    graphManager.getPoint(graphManager.getLastPointTime());
                    pos = driveTrain.getFieldTileCenter(graphManager.getXOfPoint(), graphManager.getYOfPoint()).getComponents();
                }
                graphManager.PlacePoint(pos[0], pos[1] + 24, Math.max(time, graphManager.getLastPointTime()) + 2);
            }

            if (down.OnButtonDown(gamepad1.dpad_down))
            {
                double[] pos = null;
                if (time > graphManager.getLastPointTime()) {
                    pos = driveTrain.getFieldTileCenter().getComponents();
                    graphManager.PlacePoint(pos[0], pos[1], time);
                }
                else
                {
                    graphManager.getPoint(graphManager.getLastPointTime());
                    pos = driveTrain.getFieldTileCenter(graphManager.getXOfPoint(), graphManager.getYOfPoint()).getComponents();
                }
                graphManager.PlacePoint(pos[0], pos[1] - 24, Math.max(time, graphManager.getLastPointTime()) + 2);
            }

            if (right.OnButtonDown(gamepad1.dpad_right))
            {
                double[] pos = null;
                if (time > graphManager.getLastPointTime()) {
                    pos = driveTrain.getFieldTileCenter().getComponents();
                    graphManager.PlacePoint(pos[0], pos[1], time);
                }
                else
                {
                    graphManager.getPoint(graphManager.getLastPointTime());
                    pos = driveTrain.getFieldTileCenter(graphManager.getXOfPoint(), graphManager.getYOfPoint()).getComponents();
                }
                graphManager.PlacePoint(pos[0] + 24, pos[1], Math.max(time, graphManager.getLastPointTime()) + 2);
            }

            if (left.OnButtonDown(gamepad1.dpad_left))
            {
                double[] pos = null;
                if (time > graphManager.getLastPointTime()) {
                    pos = driveTrain.getFieldTileCenter().getComponents();
                    graphManager.PlacePoint(pos[0], pos[1], time);
                }
                else
                {
                    graphManager.getPoint(graphManager.getLastPointTime());
                    pos = driveTrain.getFieldTileCenter(graphManager.getXOfPoint(), graphManager.getYOfPoint()).getComponents();
                }
                graphManager.PlacePoint(pos[0] - 24, pos[1], Math.max(time, graphManager.getLastPointTime()) + 2);
            }

            if (bX.OnButtonDown(gamepad1.x))
            {
                // get rid of all points:
                graphManager.deleteAllPoints(driveTrain.getPosition().getComponent(0), driveTrain.getPosition().getComponent(1), time);
            }

            if (time > graphManager.getLastPointTime() || (gamepad1.right_bumper))
            {
                InitialYComponent = gamepad1.left_stick_y;
                InitialXComponent = gamepad1.left_stick_x;
            } else {
                graphManager.getPoint(time);
                double[] robotPos = driveTrain.getPosition().getComponents();
                InitialXComponent = (graphManager.getXOfPoint() - robotPos[0]) * 0.5;
                InitialYComponent = (graphManager.getYOfPoint() - robotPos[1]) * 0.5;
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