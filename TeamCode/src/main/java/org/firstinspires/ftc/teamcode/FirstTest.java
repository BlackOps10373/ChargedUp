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
        DcMotor lw = hardwareMap.get(DcMotor.class, "lw");
        DcMotor rw = hardwareMap.get(DcMotor.class, "rw");
        rw.setDirection(DcMotor.Direction.REVERSE);
        DcMotor blw = hardwareMap.get(DcMotor.class, "blw");
        DcMotor brw = hardwareMap.get(DcMotor.class, "brw");

        brw.setDirection(DcMotorSimple.Direction.REVERSE);
        lw.setMode(RUN_WITHOUT_ENCODER);
        blw.setMode(RUN_WITHOUT_ENCODER);
        rw.setMode(STOP_AND_RESET_ENCODER);
        rw.setMode(RUN_WITHOUT_ENCODER);
        brw.setMode(RUN_WITHOUT_ENCODER);
        DriveTrain driveTrain = new DriveTrain(telemetry, lw, rw, blw, brw);

        waitForStart();
        while (opModeIsActive())
        {
            lw.setPower(gamepad1.left_stick_y / 2);
            rw.setPower(gamepad1.left_stick_y / 2);
            blw.setPower(gamepad1.left_stick_y / 2);
            brw.setPower(gamepad1.left_stick_y / 2);
            //driveTrain.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}
