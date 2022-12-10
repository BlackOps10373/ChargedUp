package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SharedCode.DriveTrain;
import org.firstinspires.ftc.teamcode.SharedCode.oldDriveTrain;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap);
        AILS ails = new AILS(telemetry,hardwareMap);
        //driveTrain.initMotors();
        telemetry.addData("Calibration:", "Complete");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {

            if (gamepad1.dpad_up)
                ails.grabber.setPosition(0.08);

            if (gamepad1.dpad_down)
                ails.grabber.setPosition(0);

            if(gamepad1.a)
                ails.tics = (-2500);

            if(gamepad1.b)
                ails.tics = (-2000);

            if (gamepad1.x)
                ails.tics = (-1500);

            if (gamepad1.y)
                ails.tics = (100);

            if (!(gamepad1.right_stick_y < 0 && (ails.touchSensorLeft.isPressed() || ails.touchSensorRight.isPressed())))
                ails.tics += gamepad1.right_stick_y * 10;

            ails.zip(ails.tics + ails.offsetLeft, ails.tics + ails.offsetRight);

            ails.zipChainLeft.setPower(ails.zipChainLeftPower);
            ails.zipChainRight.setPower(ails.zipChainRightPower);

            driveTrain.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            //driveTrain.gyroStraight();




        }
    }
}

