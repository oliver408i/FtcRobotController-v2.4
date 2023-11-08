/* Copyright (c) 2023 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.testers;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * This OpMode determines the best Exposure for minimizing image motion-blur on a Webcam
 * Note that it is not possible to control the exposure for a Phone Camera, so if you are using a Phone for the Robot Controller
 * this OpMode/Feature only applies to an externally connected Webcam
 *
 * The goal is to determine the smallest (shortest) Exposure value that still provides reliable Tag Detection.
 * Starting with the minimum Exposure and maximum Gain, the exposure is slowly increased until the Tag is
 * detected reliably from the likely operational distance.
 *
 *
 * The best way to run this optimization is to view the camera preview screen while changing the exposure and gains.
 *
 * To do this, you need to view the RobotController screen directly (not from Driver Station)
 * This can be done directly from a RC phone screen (if you are using an external Webcam), but for a Control Hub you must either plug an
 * HDMI monitor into the Control Hub HDMI port, or use an external viewer program like ScrCpy (https://scrcpy.org/)
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Disabled
@TeleOp(name="Optimize Tfod Exposure")

public class ExposureTester extends LinearOpMode
{
    private VisionPortal visionPortal = null;        // Used to manage the video source.
    private TfodProcessor tfod;              // Used for managing the AprilTag detection process.
    private int     myExposure  ;
    private int     minExposure ;
    private int     maxExposure ;
    private int     myGain      ;
    private int     minGain ;
    private int     maxGain ;

    boolean thisExpUp = false;
    boolean thisExpDn = false;
    boolean thisGainUp = false;
    boolean thisGainDn = false;

    boolean lastExpUp = false;
    boolean lastExpDn = false;
    boolean lastGainUp = false;
    boolean lastGainDn = false;
    @Override public void runOpMode()
    {
        // Initialize the TFOD Detection process
        initTfod();

        // Establish Min and Max Gains and Exposure.  Then set a low exposure with high gain
        getCameraSetting();
        myExposure = Math.min(5, minExposure);
        myGain = maxGain;
        setManualExposure(myExposure, myGain);

        // Wait for the match to begin.
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addLine("Find lowest Exposure that gives reliable detection.");
            telemetry.addLine("Use Left bump/trig to adjust Exposure.");
            telemetry.addLine("Use Right bump/trig to adjust Gain.\n");



            List<Recognition> currentDetections = tfod.getRecognitions();


            for (Recognition recognition : currentDetections) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }   // end for() loop


            // Display how many Tags Detected

            int numTags = currentDetections.size();


//            if (numTags > 0 )
//                telemetry.addData("Tag", "####### %d Detected  ######", currentDetections.size());
//            else
//                telemetry.addData("Tag", "----------- none - ----------");


            telemetry.addData("Exposure","%d  (%d - %d)", myExposure, minExposure, maxExposure);
            telemetry.addData("Gain","%d  (%d - %d)", myGain, minGain, maxGain);
            telemetry.update();

            // check to see if we need to change exposure or gain.
            thisExpUp = gamepad1.left_bumper;
            thisExpDn = gamepad1.left_trigger > 0.25;
            thisGainUp = gamepad1.right_bumper;
            thisGainDn = gamepad1.right_trigger > 0.25;

            // look for clicks to change exposure
            if (thisExpUp && !lastExpUp) {
                myExposure = Range.clip(myExposure + 1, minExposure, maxExposure);
                setManualExposure(myExposure, myGain);
            } else if (thisExpDn && !lastExpDn) {
                myExposure = Range.clip(myExposure - 1, minExposure, maxExposure);
                setManualExposure(myExposure, myGain);
            }

            // look for clicks to change the gain
            if (thisGainUp && !lastGainUp) {
                myGain = Range.clip(myGain + 1, minGain, maxGain );
                setManualExposure(myExposure, myGain);
            } else if (thisGainDn && !lastGainDn) {
                myGain = Range.clip(myGain - 1, minGain, maxGain );
                setManualExposure(myExposure, myGain);
            }

            lastExpUp = thisExpUp;
            lastExpDn = thisExpDn;
            lastGainUp = thisGainUp;
            lastGainDn = thisGainDn;

            sleep(20);
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.

                //.setModelFileName("object_test1.tflite")

                .setModelFileName("red_cube_v1_model_20231026_113436.tflite")

                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)


                //.setModelLabels(LABELS)

                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)


                .build();


        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");


        //Camera webcam = hardwareMap.get(Camera.class, "Webcam 1");


        //FtcDashboard.getInstance().startCameraStream(,60);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();


        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);
        tfod.setMinResultConfidence(0.5f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /*
        Manually set the camera gain and exposure.
        Can only be called AFTER calling initAprilTag();
        Returns true if controls are set.
     */
    private boolean    setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }

    /*
        Read this camera's minimum and maximum Exposure and Gain settings.
        Can only be called AFTER calling initAprilTag();
     */
    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Get camera control values unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
        }
    }
}
