package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SharedCode.DriveTrain;
import org.firstinspires.ftc.teamcode.SharedCode.oldDriveTrain;

@TeleOp(name = "HeadlessTest", group = "TeleOp")
public class HeadlessTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        oldDriveTrain driveTrain = new oldDriveTrain(telemetry, hardwareMap);
        //driveTrain.initMotors();

        waitForStart();
        while (opModeIsActive())
        {


            driveTrain.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            //driveTrain.gyroStraight();
            driveTrain.setWheelsToPowers();
        }
    }
}
