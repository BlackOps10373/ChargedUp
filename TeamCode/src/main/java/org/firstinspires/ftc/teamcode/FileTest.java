package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SharedCode.DriveTrain;
import org.firstinspires.ftc.teamcode.SharedCode.oldDriveTrain;

@TeleOp(name = "FileTest", group = "TeleOp")
public class FileTest extends LinearOpMode {

    native boolean readFile();
    native boolean testWrite();
    static {
       System.loadLibrary("ftcrobotcontroller");
    }

    class test
    {
        public int i = 0;
    }


    @Override
    public void runOpMode() {

        test T = new test();
        T.i = 1;

        test J = new test();
        J = T;
        J.i = 230;

        telemetry.addData("num", T.i);
        telemetry.update();

        /*
        boolean OpenedInWrite = testWrite();
        telemetry.addData("File opened in write?", OpenedInWrite);

        boolean readBack = readFile();
        telemetry.addData("Read back some correctly?", readBack);
        telemetry.update();


         */
        waitForStart();
    }
}

