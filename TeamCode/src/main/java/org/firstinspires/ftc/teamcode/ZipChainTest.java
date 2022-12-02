package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


    @TeleOp(name = "ZipChainTest", group = "TeleOp")
    public class ZipChainTest extends LinearOpMode {

        @Override
        public void runOpMode() {

            DcMotor zipChain = hardwareMap.get(DcMotor.class, "zipChain");
            zipChain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            waitForStart();
            while (opModeIsActive())
            {

                zipChain.setPower(gamepad1.right_stick_y);

                //driveTrain.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }
        }
    }
