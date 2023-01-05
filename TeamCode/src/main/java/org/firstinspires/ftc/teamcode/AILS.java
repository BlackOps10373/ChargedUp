package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class AILS {
        Telemetry telemetry;

        Servo grabber;

        DcMotor zipChainLeft;
        DcMotor zipChainRight;

        TouchSensor touchSensorLeft;
        TouchSensor touchSensorRight;

        double zipChainLeftPower = 0;
        double zipChainRightPower = 0;

        int tics = 0;
        int offsetLeft = 0;
        int offsetRight = 0;

        public AILS (Telemetry t, HardwareMap hardwareMap){
            telemetry   = t;

            grabber = hardwareMap.get(Servo.class, "grabber");
            grabber.setPosition(0);

            zipChainLeft  = hardwareMap.get(DcMotor.class, "zipChainLeft");
            zipChainLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            zipChainLeft.setTargetPosition(0);
            zipChainLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            zipChainLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            zipChainLeft.setDirection(DcMotor.Direction.REVERSE);

            zipChainRight   = hardwareMap.get(DcMotor.class, "zipChainRight");
            zipChainRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            zipChainRight.setTargetPosition(0);
            zipChainRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            zipChainRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            touchSensorLeft = hardwareMap.get(TouchSensor.class, "touchSensorLeft");
            touchSensorRight = hardwareMap.get(TouchSensor.class, "touchSensorRight");
        }

        public void zip (int ticsLeft, int ticsRight) {
            zipChainLeft.setTargetPosition(ticsLeft);
            zipChainRight.setTargetPosition(ticsRight);
            zipChainLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            zipChainRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (zipChainLeft.isBusy() || zipChainRight.isBusy()) {
                int[] ticks = new int[2];
                ticks[0] = Math.abs(zipChainLeft.getTargetPosition() - zipChainLeft.getCurrentPosition());
                ticks[1] = Math.abs(zipChainRight.getTargetPosition() - zipChainRight.getCurrentPosition());
                Arrays.sort(ticks);

                //zipChainLeft.setPower((zipChainLeft.getTargetPosition() - zipChainLeft.getCurrentPosition()) * 1.0 / ticks[1]);
                //zipChainRight.setPower((zipChainRight.getTargetPosition() - zipChainRight.getCurrentPosition()) * 1.0 / ticks[1]);
                zipChainLeft.setPower(1);
                zipChainRight.setPower(1);

                /*if (ticks[0] < 300){
                    if(touchSensorLeft.isPressed()) {
                        offsetLeft = zipChainLeft.getCurrentPosition();
                        zipChainLeftPower = 0;
                    }
                    else
                        zipChainLeftPower = ((zipChainLeft.getTargetPosition() - zipChainLeft.getCurrentPosition()) * 1.0 / ticks[1] / 2);
                    if(touchSensorRight.isPressed()) {
                        offsetRight= zipChainLeft.getCurrentPosition();
                        zipChainRightPower = 0;
                    }
                    else
                        zipChainRightPower = ((zipChainRight.getTargetPosition() - zipChainRight.getCurrentPosition()) * 1.0 / ticks[1] / 2);
                }
                else {
                    zipChainLeftPower = ((zipChainLeft.getTargetPosition() - zipChainLeft.getCurrentPosition()) * 1.0 / ticks[1]);
                    zipChainRightPower = ((zipChainRight.getTargetPosition() - zipChainRight.getCurrentPosition()) * 1.0 / ticks[1]);
                }*/
            }
        }

    public void unzip () {
        int[] ticks = new int[2];
        zipChainLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        zipChainRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        zipChainLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        zipChainRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticks[0] = Math.abs(zipChainLeft.getCurrentPosition());
        ticks[1] = Math.abs(zipChainRight.getCurrentPosition());
        Arrays.sort(ticks);

        zipChainLeft.setPower(-zipChainLeft.getCurrentPosition() * 1.0 / ticks[1]);
        zipChainRight.setPower(-zipChainRight.getCurrentPosition() * 1.0 / ticks[1]);

    }




}
