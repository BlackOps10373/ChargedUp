package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.File;

@TeleOp(name = "RecordInputs", group = "TeleOp")
public class RecordInputs extends LinearOpMode {

    byte[] recordingBytes = new byte[2000];

    boolean recordBytesToFile(File fileToRecordTo)
    {
        // returns false on fail
        FileOutputStream fileOutput;
        try {
            fileOutput = new FileOutputStream(fileToRecordTo);
        }
        catch (java.io.FileNotFoundException exception)
        {
            telemetry.addData("failed", "Io exception");
            telemetry.update();
            return false;
        }

        try {
            fileOutput.write(recordingBytes);
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

    @Override
    public void runOpMode() {
        waitForStart();
        recordingBytes[2] = 64;
        recordingBytes[0] = 0;
        recordingBytes[1] = 0;

        File recordFile = AppUtil.getInstance().getSettingsFile("RecordedInputs.txt");

        recordBytesToFile(recordFile);
    }
}
