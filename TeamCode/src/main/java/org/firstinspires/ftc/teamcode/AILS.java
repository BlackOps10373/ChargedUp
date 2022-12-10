package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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


        public AILS (Telemetry t, HardwareMap hardwareMap){
            telemetry   = t;

            grabber = hardwareMap.get(Servo.class, "grabber");
            grabber.setPosition(0);

            zipChainLeft  = hardwareMap.get(DcMotor.class, "zipChainLeft");
            zipChainLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            zipChainLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            zipChainRight   = hardwareMap.get(DcMotor.class, "zipChainRight");
            zipChainRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            zipChainRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            touchSensorLeft = hardwareMap.get(TouchSensor.class, "touchSensorLeft");
            touchSensorRight = hardwareMap.get(TouchSensor.class, "touchSensorRight");
        }

        public void zipUp (int tics) {
            int[] ticks = new int[2];
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
            }
            zipChainLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            zipChainRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
