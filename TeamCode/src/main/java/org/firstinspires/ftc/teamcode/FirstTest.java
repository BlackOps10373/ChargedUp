package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.SharedCode.DriveTrain;
import org.firstinspires.ftc.teamcode.SharedCode.oldDriveTrain;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class FirstTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap);
        driveTrain.initMotors();

        waitForStart();
        while (opModeIsActive())
        {
            driveTrain.lw.setPower(gamepad1.left_stick_y / 2);
            driveTrain.rw.setPower(gamepad1.left_stick_y / 2);
            driveTrain.blw.setPower(gamepad1.left_stick_y / 2);
            driveTrain.brw.setPower(gamepad1.left_stick_y / 2);
            //driveTrain.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}
