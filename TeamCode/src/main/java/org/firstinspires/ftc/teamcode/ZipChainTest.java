package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.SharedCode.Button;

    @TeleOp(name = "ZipChainTest", group = "TeleOp")
    public class ZipChainTest extends LinearOpMode {

        Button btnX = new Button();

        @Override
        public void runOpMode() {

            DcMotor zipChainLeft = hardwareMap.get(DcMotor.class, "zipChainLeft");
            zipChainLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            DcMotor zipChainRight = hardwareMap.get(DcMotor.class, "zipChainRight");
            zipChainRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            zipChainRight.setDirection(DcMotorSimple.Direction.REVERSE);



            waitForStart();
            while (opModeIsActive())
            {

                //zipChain.setPower(gamepad1.right_stick_y);

                if(btnX.OnButtonDown(gamepad1.x))
                {
                    // OnButtonDown is required to call for OnToggleOn to work
                    if(btnX.OnToggleOn())
                    {
                        zipChainLeft.setPower(0.3);
                        zipChainRight.setPower(0.3);
                    }
                    else
                    {
                        zipChainLeft.setPower(0);
                        zipChainRight.setPower(0);
                    }
                }


                if (gamepad1.a)
                {
                    // emergency stop for if my toggle code does not work
                    zipChainLeft.setPower(0);
                    zipChainRight.setPower(0);
                }


                telemetry.addData("On", btnX.OnToggleOn());
                telemetry.addData("TicksLeft", zipChainLeft.getCurrentPosition());
                telemetry.addData("TicksRight", zipChainRight.getCurrentPosition());
                telemetry.update();
                //driveTrain.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }
        }
    }
