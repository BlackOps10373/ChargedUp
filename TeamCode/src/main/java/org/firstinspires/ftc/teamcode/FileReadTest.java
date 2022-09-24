package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;

import java.nio.ByteBuffer;

@TeleOp(name = "FileReadTest", group = "TeleOp")
public class FileReadTest extends LinearOpMode {

    int sizeOfArray = 2000;

    boolean setByteArrayToFile(File file, byte[] bytes)
    {
        FileOutputStream fileOutput;
        try {
            fileOutput = new FileOutputStream(file);
        }
        catch (java.io.FileNotFoundException exception)
        {
            telemetry.addData("failed", "Io exception");
            telemetry.update();
            return false;
        }

        try {
            fileOutput.write(bytes);
            telemetry.addData("YES", "Wrote bytes");
            fileOutput.flush(); // remember to call flush. It does not work without it.
            fileOutput.close();
            telemetry.update();
        }
        catch (java.io.IOException exception)
        {
            telemetry.addData("failed", "Io exception");
            telemetry.update();
            return false;
        }
        return true;
    }

    byte[] getByteArrayFromFile(File file)
    {
        byte[] bytes = new byte[sizeOfArray];

        FileInputStream fileInput;
        try {
            fileInput = new FileInputStream(file);
        }
        catch (java.io.FileNotFoundException exception)
        {
            telemetry.addData("failed", "FileNotFound exception");
            telemetry.update();
            waitForStart();
            return bytes;
        }

        try {
            fileInput.read(bytes);
            telemetry.addData("YES", "Read bytes");
            fileInput.close();
        }
        catch (java.io.IOException exception) {
            telemetry.addData("failed", "Io exception");
            telemetry.update();
            waitForStart();
            return bytes;
        }
        return bytes;
    }



    byte[] separateIntTo4Bytes(int theInt)
    {
        byte[] eachByte = new byte[4];
        eachByte[0] = (byte) (theInt & 0x000000ff);
        eachByte[1] = (byte) (theInt & 0x0000ff00);
        eachByte[2] = (byte) (theInt & 0x00ff0000);
        eachByte[3] = (byte) (theInt & 0xff000000);
        return eachByte;
    }

    float getFloatFromByteArray(byte[] bytes, int offsetFloatNumber)
    {
        // Example of how I kind of want the file data functions to work
        offsetFloatNumber *= 4;

        telemetry.addData("inputBytes", bytes);

        ByteBuffer bB = ByteBuffer.wrap(bytes, offsetFloatNumber, 4);
        float returnFloat = bB.getFloat(0);
        telemetry.addData("getFloat", returnFloat);
        return returnFloat;

    }

    void setFloatInByteArray(byte[] bytes, float f, int offsetFloatNumber)
    {
        // Example of how I kind of want the file data functions to work
        offsetFloatNumber *= 4;

        byte[] bytesOfFloat = separateIntTo4Bytes(Float.floatToIntBits(f));
        bytes[offsetFloatNumber] = bytesOfFloat[0];
        bytes[offsetFloatNumber + 1] = bytesOfFloat[1];
        bytes[offsetFloatNumber + 2] = bytesOfFloat[2];
        bytes[offsetFloatNumber + 3] = bytesOfFloat[3];
    }

    @Override
    public void runOpMode() {


        //waitForStart();

        File recordFile = AppUtil.getInstance().getSettingsFile("FolderTest.txt");

        byte[] recordingBytes = getByteArrayFromFile(recordFile);

        //byte[] recordingBytes = new byte[20];
        ByteBuffer byteBuffer = ByteBuffer.wrap(recordingBytes, 0, recordingBytes.length);

        //byteBuffer.putFloat(0, 2.0f);

        //byteBuffer.putFloat(4, 2.1f);

        //setByteArrayToFile(recordFile, recordingBytes);


        telemetry.addData("the value in 0", byteBuffer.getFloat(0));
        telemetry.addData("the value in 1", byteBuffer.getFloat(4));
        telemetry.update();
        waitForStart();

    }
}
