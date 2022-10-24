package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SharedCode.DriveTrain;

@TeleOp(name = "RawWheelSpeeds", group = "TeleOp")
public class RawWheelSpeeds extends LinearOpMode {

    @Override
    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap);
        driveTrain.initMotors();

        waitForStart();
        while (opModeIsActive())
        {
            /*
            driveTrain.lw.setPower(gamepad1.left_stick_y / 2);
            driveTrain.rw.setPower(gamepad1.left_stick_y / 2);
            driveTrain.blw.setPower(gamepad1.left_stick_y / 2);
            driveTrain.brw.setPower(gamepad1.left_stick_y / 2);
*/

            //telemetry.update();
            driveTrain.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}
