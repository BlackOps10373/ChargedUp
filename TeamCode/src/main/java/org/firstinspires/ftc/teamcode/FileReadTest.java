package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp(name = "FileReadTest", group = "TeleOp")
public class FileReadTest extends LinearOpMode {

    int combine4Bytes(char c1, char c2, char c3, char c4)
    {
        return (int)c1 | ((int)c2 << 8) | ((int)c3 << 16) | ((int)c4 << 24);
    }

    float getFloatFromFile(File file, int offsetFloatNumber)
    {
        // Example of how I kind of want the file data functions to work
        offsetFloatNumber *= 4;
        String AsString = ReadWriteFile.readFile(file);
        char[] stringBytes = AsString.toCharArray();

        int theIntFloatBits = combine4Bytes(stringBytes[offsetFloatNumber], stringBytes[offsetFloatNumber + 1], stringBytes[offsetFloatNumber + 2], stringBytes[offsetFloatNumber + 3]);

        return Float.intBitsToFloat(theIntFloatBits);
    }

    @Override
    public void runOpMode() {


        //waitForStart();
        byte t = 1;

        File recordFile = AppUtil.getInstance().getSettingsFile("FolderTest.txt");

        //float TheFloatIAmWriting = 2.5f;

        float theFloat = getFloatFromFile(recordFile, 0);
        /*
        stringBytes[0] =  (char)(theIntIAmWriting & 0x000000ff);
        stringBytes[1] = (char)((theIntIAmWriting & 0x0000ff00) >> 8);
        stringBytes[2] = (char)((theIntIAmWriting & 0x00ff0000) >> 16);
        stringBytes[3] = (char)((theIntIAmWriting & 0xff000000) >> 24);
        */


        telemetry.addData("int", theFloat);
        telemetry.update();
        waitForStart();
    }
}
