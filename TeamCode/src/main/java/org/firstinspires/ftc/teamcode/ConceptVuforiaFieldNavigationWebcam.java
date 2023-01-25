/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import android.app.ApplicationErrorReport;
import android.os.BatteryManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.BatteryChecker;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotserver.internal.webserver.RobotControllerWebHandlers;
import org.firstinspires.ftc.teamcode.SharedCode.DriveTrain;

import java.util.ArrayList;
import java.util.List;

/**
 * This OpMode illustrates using the Vuforia localizer to determine positioning and orientation of
 * robot on the FTC field using a WEBCAM.  The code is structured as a LinearOpMode
 *
 * NOTE: If you are running on a Phone with a built-in camera, use the ConceptVuforiaFieldNavigation example instead of this one.
 * NOTE: It is possible to switch between multiple WebCams (eg: one for the left side and one for the right).
 *       For a related example of how to do this, see ConceptTensorFlowObjectDetectionSwitchableCameras
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * Finally, the location of the camera on the robot is used to determine the
 * robot's location and orientation on the field.
 *
 * To learn more about the FTC field coordinate model, see FTC_FieldCoordinateSystemDefinition.pdf in this folder
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@TeleOp(name="Vuforia Field Nav Webcam", group ="Concept")
//@Disabled
public class ConceptVuforiaFieldNavigationWebcam extends LinearOpMode {

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AZvk+Kr/////AAABmam7AIU85E2Xlvyy/SSUUO9XZkmjFDE33B9YG/yTs5x3qJcLm1WtxVLNjx53GEMOe0/mBh7q1to2EyApYoHB7f1hoVNQL9k+09+ivKVDbxv4lXKyWKNJiV2WrQVwCpqm4L1cS+02zbA4tUfXF8MFpr9NDfkeywbd3D9PHeeQddokGZQr6lOKxVymt0aMSmtSvZgEJOSAMUCRoW/bj85chY1Z2aAFRyldbOuCTeaXK9Y+SYHi+RdYc4ZMI20nsMpzxICKJFI0p28hK9D+YFTNnvkCU9dMcqz2Rx8lDrGklWqMP9NOu8CV7avoYgIcKyyM38L6bCk67v/SFCspcOAdpYJRlmnzR2isxzwjOhbxkKna";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation   = null;
    private VuforiaLocalizer vuforia    = null;
    private VuforiaTrackables targets   = null ;
    private WebcamName webcamName       = null;

    private boolean targetVisible       = false;

    OpenGLMatrix predictedLocation = OpenGLMatrix.translation(0, 0,0);

    @Override public void runOpMode() {
        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        webcamName = hardwareMap.get(WebcamName.class, "logiCam");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera-preview is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera we wish to use.
        parameters.cameraName = webcamName;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("PowerPlay");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Name and locate each trackable object
        identifyTarget(0, "Red Audience Wall",   -halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0,  90);
        identifyTarget(1, "Red Rear Wall",        halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
        identifyTarget(2, "Blue Audience Wall",  -halfField,   oneAndHalfTile, mmTargetHeight, 90, 0,  90);
        identifyTarget(3, "Blue Rear Wall",       halfField,   oneAndHalfTile, mmTargetHeight, 90, 0, -90);

        /*
         * Create a transformation matrix describing where the camera is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
         * with the wide (horizontal) axis of the camera aligned with the X axis, and
         * the Narrow (vertical) axis of the camera aligned with the Y axis
         *
         * But, this example assumes that the camera is actually facing forward out the front of the robot.
         * So, the "default" camera position requires two rotations to get it oriented correctly.
         * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
         * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
         *
         * Finally the camera can be translated to its actual mounting position on the robot.
         *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
         */

        final float CAMERA_FORWARD_DISPLACEMENT  = 6.5f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 5.25f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = -5.75f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens
        final float CAMERA_ROLL_ROTATION_DEG         = 90.0f;

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90 + CAMERA_ROLL_ROTATION_DEG, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            // Note: the camera transformation matrix might need to be inverted before using it
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        /*
         * WARNING:
         * In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
         * This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
         * CONSEQUENTLY do not put any driving commands in this loop.
         * To restore the normal opmode structure, just un-comment the following line:
         */

        // waitForStart();

        /* Note: To use the remote camera preview:
         * AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
         * Tap the preview window to receive a fresh image.
         * It is not permitted to transition to RUN while the camera preview window is active.
         * Either press STOP to exit the OpMode, or use the "options menu" again, and select "Camera Stream" to close the preview window.
         */

        boolean moveRealRobot = true;
        boolean useViewforia = true;

        float speed = 0.003f; // (whatever units the matrix uses by default) a second
        float correcterMult = 0.1f;
        double lastTime = time;
        OpenGLMatrix LControl_stick_input = null;
        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap);

        //driveTrain.lw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveTrain.rw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveTrain.blw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveTrain.brw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targets.activate();
        while (!isStopRequested()) {
            double deltaTime = time - lastTime;
            // update the predicted location based on controller input
            //VectorF LControl_stick_input = new VectorF(-gamepad1.left_stick_x * speed * (float)deltaTime,-gamepad1.left_stick_y * speed * (float)deltaTime,0, 0);

            // double check the x and y axis mapping from the controller
            {
                // Make the controller input into a vector to do vector math
                VectorF vecLControl = new VectorF(gamepad1.left_stick_x * speed * (float) deltaTime, -gamepad1.left_stick_y * speed * (float) deltaTime, 0);
                //if (vecLControl.magnitude() > 1.0f)
                    //vecLControl = vecLControl.normalized3D(); // clamp the magnitude to 1

                // Convert input vector into a translation matrix to multiply with the robot's transformation matrix
                LControl_stick_input = OpenGLMatrix.translation(vecLControl.get(0), vecLControl.get(1), vecLControl.get(2));
            }
            if (moveRealRobot)
                driveTrain.rawMove(gamepad1.left_stick_x / 2, gamepad1.left_stick_y / 2, gamepad1.right_stick_x / 2);

            if (gamepad1.a)
            {
                moveRealRobot = false;
                useViewforia = false;
            }
            if (gamepad1.b)
            {
                moveRealRobot = true;
                useViewforia = true;
            }

            telemetry.addData("LControl stick magnitude", LControl_stick_input.getTranslation().magnitude());
            telemetry.addData("rawLstickX", gamepad1.right_stick_x);
            telemetry.addData("rawLstickY", gamepad1.left_stick_y);

            if(predictedLocation != null && (LControl_stick_input.getTranslation().magnitude() > 0.1)) {
                predictedLocation = predictedLocation.multiplied(LControl_stick_input);
            }
            //LControl_stick_input = OpenGLMatrix.rotation(DEGREES,Orientation.getOrientation(predictedLocation,
            //        EXTRINSIC, XYZ, DEGREES).thirdAngle,0,0,0).multiplied(LControl_stick_input);

            // check all the trackable targets to see which one (if any) is visible.

            targetVisible = false;
            if (useViewforia) {
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {


                            if (deltaTime < 1.0f && LControl_stick_input.getTranslation().magnitude() > 0.3) // only updated speed if deltaTime is less than 1 second
                            {
                                speed = ((robotLocationTransform.getTranslation().subtracted(lastLocation).getTranslation().magnitude())
                                        / (float) deltaTime) / LControl_stick_input.getTranslation().magnitude();
                            }
                            lastLocation = robotLocationTransform;
                            predictedLocation = new OpenGLMatrix(lastLocation);
                            lastTime = time;
                        }
                        break;
                    }
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (predictedLocation != null) {
                // express position (translation) of robot in inches.
                VectorF translation = predictedLocation.getTranslation();
                telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(predictedLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targets.deactivate();
    }

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }
}
