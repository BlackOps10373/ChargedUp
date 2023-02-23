package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SharedCode.DriveTrain;
import org.firstinspires.ftc.teamcode.SharedCode.oldDriveTrain;
import org.opencv.core.Mat;

@TeleOp(name = "BlueSideTeleOp", group = "TeleOp")
public class BlueSideTeleOp extends LinearOpMode {

    public Vector2D leftStick;
    public Vector2D rightStick;
    public Vector2D motorSpeeds;

    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap);
        //while (!driveTrain.imu.isGyroCalibrated()) {
            telemetry.addLine("Robot Is Initializing: DO NOT START");
            telemetry.update();
        //}
        telemetry.addLine("Initialization Complete: Ready To Start");
        telemetry.update();
        waitForStart();
        //leftStick = new Vector2D((double) gamepad1.left_stick_x, (double) -gamepad1.left_stick_y);
        //rightStick = new Vector2D((double) gamepad1.right_stick_x, (double) gamepad1.right_stick_y);
        while(opModeIsActive() && !isStopRequested()){

            driveTrain.updatePosition();

            telemetry.addData("Heading", driveTrain.degreeCalc180(driveTrain.getHeading()*-180));
            telemetry.addData("LeftEncoder", driveTrain.leftEncoderMotor.getCurrentPosition());
            telemetry.addData("RightEncoder", driveTrain.rightEncoderMotor.getCurrentPosition());
            telemetry.addData("CenterEncoder", driveTrain.centerEncoderMotor.getCurrentPosition());

            telemetry.addData("Y value", driveTrain.getY());
            telemetry.addData("X value", driveTrain.getX());
            telemetry.addData("Position", driveTrain.getPosition());
            telemetry.update();

            /*driveTrain.lw.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            driveTrain.rw.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            driveTrain.blw.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
            driveTrain.brw.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);*/
            driveTrain.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
}