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
            driveTrain.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            //driveTrain.gyroStraight();
        }
    }
}

