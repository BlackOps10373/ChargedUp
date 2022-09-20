package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;

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
        byte[] recordingBytes = new byte[2000];
        recordingBytes[2] = 0;

        File recordFile = AppUtil.getInstance().getSettingsFile("FolderTest.txt");

        FileInputStream fileInput;
        try {
            fileInput = new FileInputStream(recordFile);
        }
        catch (java.io.FileNotFoundException exception)
        {
            telemetry.addData("failed", "FileNotFound exception");
            telemetry.update();
            waitForStart();
            return;
        }

        try {
            fileInput.read(recordingBytes);
            telemetry.addData("YES", "Read bytes");
            fileInput.close();
        }
        catch (java.io.IOException exception) {
            telemetry.addData("failed", "Io exception");
            telemetry.update();
            waitForStart();
            return;
        }
        telemetry.addData("the value I stored", recordingBytes[2]);
        telemetry.update();
        waitForStart();

    }
}
