package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SharedCode.DriveTrain;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

@TeleOp(name = "FileWriteTest", group = "TeleOp")
public class FileTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        byte t = 1;

        File recordFile = AppUtil.getInstance().getSettingsFile("FolderTest.txt");

        float TheFloatIAmWriting = -2.5f;


        char[] stringBytes = new char[20];

        int theIntIAmWriting = Float.floatToIntBits(TheFloatIAmWriting);
        stringBytes[0] =  (char)(theIntIAmWriting & 0x000000ff);
        stringBytes[1] = (char)((theIntIAmWriting & 0x0000ff00) >> 8);
        stringBytes[2] = (char)((theIntIAmWriting & 0x00ff0000) >> 16);
        stringBytes[3] = (char)((theIntIAmWriting & 0xff000000) >> 24);

        String AsString = String.copyValueOf(stringBytes);
        ReadWriteFile.writeFile(recordFile, AsString);


        telemetry.addData("int", stringBytes[0]);
        telemetry.update();
    }
}
