package org.firstinspires.ftc.teamcode.SharedCode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ObjectGrab {
    Telemetry telemetry;

    CRServo leftGrab;
    CRServo rightGrab;

    DcMotor upDownMotor;
    DcMotor rotate;

    TouchSensor touchSensor;

    int armTargetPosition = 0;
    int rotateTargetPosition = 0;

    boolean useSafeRotate = false;
    int safeRotateTargetPosition = 0;
    boolean useSafeArmTarget = false;
    int safeArmTargetPosition = 0;

    public ObjectGrab(Telemetry t, HardwareMap hardwareMap){
        telemetry   = t;
        rightGrab  = hardwareMap.get(CRServo.class, "rightGrab");
        leftGrab   = hardwareMap.get(CRServo.class, "leftGrab");

        rotate     = hardwareMap.get(DcMotor.class, "rotate");
        upDownMotor= hardwareMap.get(DcMotor.class, "upDownMotor");
        touchSensor= hardwareMap.get(TouchSensor.class, "touchSensor");
        upDownMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void armMovementCheck()
    {
        // Checks to see if the safe variables are set, checks if arm can move, then moves the arm
        if(useSafeRotate)
        {
            if(upDownMotor.getCurrentPosition() >= 2500) {
                rotateTargetPosition = safeRotateTargetPosition;
                useSafeRotate = false;
                rotate.setTargetPosition(rotateTargetPosition);
                rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotate.setPower(1);
            }
            else
            {
                if(armTargetPosition < 2500 || (useSafeArmTarget && (safeArmTargetPosition < 2500))) {
                    armTargetPosition = 5400;
                    upDownMotor.setTargetPosition(armTargetPosition);
                    upDownMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    upDownMotor.setPower(.75);
                }
            }
        }
        if(useSafeArmTarget)
        {
            if(safeArmTargetPosition >= 2500)
            {
                armTargetPosition = safeArmTargetPosition;
                upDownMotor.setTargetPosition(armTargetPosition);
                upDownMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                upDownMotor.setPower(.75);
            }
            else {
                if ((rotate.getCurrentPosition() > -300 && rotate.getCurrentPosition() < 300)) {
                    armTargetPosition = safeArmTargetPosition;
                    useSafeArmTarget = false;
                    upDownMotor.setTargetPosition(armTargetPosition);
                    upDownMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    upDownMotor.setPower(.75);
                } else {
                    useSafeRotate = true;
                    if(safeArmTargetPosition <= -300 || safeArmTargetPosition >= 300)
                        safeRotateTargetPosition = 0;
                }
            }
        }
    }

    public void safeArmMovement(int upDown, int rotation)
    {
        // set the safe arm movement variables
        if(rotation != rotateTargetPosition) {
            useSafeRotate = true;
            safeRotateTargetPosition = rotation;
        }
        if(upDown != armTargetPosition)
        {
            useSafeArmTarget = true;
            safeArmTargetPosition = upDown;
        }
    }
}
