package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SharedCode.DriveTrain;
import org.firstinspires.ftc.teamcode.SharedCode.oldDriveTrain;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        double leftStickYBefore = 0;
        double leftStickYAfter = 0;
        double leftStickXBefore = 0;
        double leftStickXAfter = 0;
        double rightStickXBefore = 0;
        double rightStickXAfter = 0;

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap);
        AILS ails = new AILS(telemetry,hardwareMap);
        int leftTics = 0;
        int rightTics = 0;
        //driveTrain.initMotors();
        telemetry.addData("Calibration:", "Complete");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("gh:", driveTrain.getHeading());
            telemetry.update();

            if (gamepad1.dpad_up)
                ails.grabber.setPosition(0.08);

            if (gamepad1.dpad_down)
                ails.grabber.setPosition(0);

            /*if(gamepad1.a)
                ails.tics = (-2500);

            if(gamepad1.b)
                ails.tics = (-2000);

            if (gamepad1.x)
                ails.tics = (-1500);

            if (gamepad1.y)
                ails.tics = (100);

            if (!(gamepad1.right_stick_y < 0 && (ails.touchSensorLeft.isPressed() || ails.touchSensorRight.isPressed())))
                ails.tics += gamepad1.right_stick_y * 40;*/

            //ails.zip(ails.tics + ails.offsetLeft, ails.tics + ails.offsetRight);
            //ails.zipChainLeft.setPower(ails.zipChainLeftPower);
            //ails.zipChainRight.setPower(ails.zipChainRightPower);
            leftTics += gamepad1.right_stick_y * 40;
            if (leftTics < -2800)
                leftTics = -2800;
            rightTics += gamepad1.right_stick_y * 40;
            if (rightTics < -2800)
                rightTics = -2800;

            ails.zip(leftTics, rightTics);




           /* if(ails.zipChainLeft.getCurrentPosition() > -2800 || gamepad1.right_stick_y > 0)
                ails.zipChainLeft.setPower(gamepad1.right_stick_y);
            else
                ails.zipChainLeft.setPower(0);

            if(ails.zipChainRight.getCurrentPosition() > -2800 || gamepad1.right_stick_y > 0)
                ails.zipChainRight.setPower(gamepad1.right_stick_y);
            else
                ails.zipChainRight.setPower(0); */

            if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0)
            {
                leftStickXAfter /= 2;
                leftStickYAfter /= 2;
            }
            else {
                leftStickYAfter = (gamepad1.left_stick_y * .05) + (leftStickYAfter * .95);
                leftStickXAfter = (gamepad1.left_stick_x * .05) + (leftStickXAfter * .95);
                rightStickXAfter = (gamepad1.right_stick_x * .05) + (rightStickXAfter * .95);
            }
            driveTrain.rw.setPower(-leftStickYAfter - leftStickXAfter - rightStickXAfter);
            driveTrain.brw.setPower(-leftStickYAfter + leftStickXAfter - rightStickXAfter);
            driveTrain.lw.setPower(-leftStickYAfter + leftStickXAfter + rightStickXAfter);
            driveTrain.blw.setPower(-leftStickYAfter - leftStickXAfter + rightStickXAfter);



            //driveTrain.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            //driveTrain.gyroStraight();




        }
    }
}

