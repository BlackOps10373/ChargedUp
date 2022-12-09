package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SharedCode.Button;

import java.util.Arrays;

@TeleOp(name = "ZipChainTest", group = "TeleOp")
    public class ZipChainTest extends LinearOpMode {

        Button btnX = new Button();

        @Override
        public void runOpMode() {


            Servo Grabber = hardwareMap.get(Servo.class, "grabber");
            Grabber.setPosition(0);
            DcMotor zipChainLeft = hardwareMap.get(DcMotor.class, "zipChainLeft");
            zipChainLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            DcMotor zipChainRight = hardwareMap.get(DcMotor.class, "zipChainRight");
            zipChainRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            zipChainLeft.setDirection(DcMotorSimple.Direction.REVERSE);



            waitForStart();
            while (opModeIsActive())
            {
                    if(gamepad1.a) {
                        int[] ticks = new int[2];
                        zipChainLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        zipChainRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        zipChainLeft.setTargetPosition(-2500);
                        zipChainRight.setTargetPosition(-2500);
                        zipChainLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        zipChainRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        while (zipChainLeft.isBusy() || zipChainRight.isBusy()) {


                            ticks[0] = Math.abs(zipChainLeft.getTargetPosition() - zipChainLeft.getCurrentPosition());
                            ticks[1] = Math.abs(zipChainRight.getTargetPosition() - zipChainRight.getCurrentPosition());
                            Arrays.sort(ticks);

                            zipChainLeft.setPower((zipChainLeft.getTargetPosition() - zipChainLeft.getCurrentPosition()) * 1.0 / ticks[1]);
                            zipChainRight.setPower((zipChainRight.getTargetPosition() - zipChainRight.getCurrentPosition()) * 1.0 / ticks[1]);

                            telemetry.addData("left ticks:", zipChainLeft.getCurrentPosition());
                            telemetry.addData("right ticks:", zipChainRight.getCurrentPosition());
                            telemetry.addData("ticks[1]", zipChainLeft.getTargetPosition() - zipChainLeft.getCurrentPosition());
                            telemetry.addData("left power:", (zipChainLeft.getTargetPosition() - zipChainLeft.getCurrentPosition()) / ticks[1]);
                            telemetry.addData("right power:", (zipChainRight.getTargetPosition() - zipChainRight.getCurrentPosition()) / ticks[1]);
                            telemetry.update();
                        }
                        zipChainLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        zipChainRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    if(gamepad1.b) {
                        Grabber.setPosition(.08);
                    }
                    zipChainLeft.setPower(gamepad1.right_stick_y);
                    zipChainRight.setPower(gamepad1.right_stick_y);
                telemetry.addData("left power:", gamepad1.right_stick_y);
                telemetry.addData("right power:", gamepad1.right_stick_y);
                telemetry.update();



                //zipChainLeft.setPower(gamepad1.right_stick_y);
                //zipChainRight.setPower(gamepad1.right_stick_y);


               /* //zipChain.setPower(gamepad1.right_stick_y);

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

                */
            }
        }
    }
