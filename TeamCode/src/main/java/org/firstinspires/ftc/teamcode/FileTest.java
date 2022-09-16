package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SharedCode.DriveTrain;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

@TeleOp(name = "FileTest", group = "TeleOp")
public class FileTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        byte t = 1;

        File recordFile = AppUtil.getInstance().getSettingsFile("..\\Movement_Recordings\\RedDuck.txt");


        // Maybe make it a char[] so it is every other 2 char rather than every other 4 bytes
        byte[] stringBytes = "This is a test string".getBytes();

        ReadWriteFile.writeFile(recordFile, stringBytes.toString());

        telemetry.addData(stringBytes.toString(), 1);
    }
}
