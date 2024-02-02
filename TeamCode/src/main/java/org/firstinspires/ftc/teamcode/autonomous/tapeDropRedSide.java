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

package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * This 2023-2024 OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "redside tape drop", group = "auton")

public class tapeDropRedSide extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * {@link #tfod} is the variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {

        initTfod();
        robot.init(hardwareMap);
        sleep(2500);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        ExposureControl exposureControl;
        GainControl gainControl;
        String cubePosition = "";

        while(!opModeIsActive()){
            if(isStopRequested()){
                visionPortal.close();
            }
            telemetryTfod();

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }
            exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            //exposureControl.setMode(ExposureControl.Mode.ContinuousAuto); // prev continuousAuto
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure((long) 1, TimeUnit.MILLISECONDS);


            gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(255);

            exposureControl.setExposure((long) 0, TimeUnit.MILLISECONDS); //prev 655


                /*telemetry.addData("exposure min: ", exposureControl.getMinExposure(TimeUnit.SECONDS));
                telemetry.addData("exposure max: ", exposureControl.getMaxExposure(TimeUnit.SECONDS));
                telemetry.addData("exposiure :", exposureControl.getExposure(TimeUnit.SECONDS));
                telemetry.update();*/

            // Share the CPU.
            sleep(20);

            List<Recognition> currentRecognitions = tfod.getRecognitions();

            // Step through the list of recognitions and display info for each one.
            double highestConf = 0;
            Recognition recognition = null;
            double x = 0;
            double y = 0;
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            for(int i = 0; i<currentRecognitions.size(); i++){
                if(currentRecognitions.get(i).getConfidence() > highestConf){
                    recognition = currentRecognitions.get(i);
                    highestConf = recognition.getConfidence();
                }
            }
            if(recognition != null) {
                x = (recognition.getLeft() + recognition.getRight()) / 2;
                y = (recognition.getTop() + recognition.getBottom()) / 2;

                if (0 < x && x < 400) {
                    cubePosition = "left";
                } else if (500 < x && x < 900) {
                    cubePosition = "center";
                } else if (1000 < x && x < 1280) {
                    cubePosition = "right";
                }
            }

        }

        visionPortal.close();


        //important: code copied from canvasBlueTeamAuto

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory temp = null;

        ArrayList<Trajectory> lotsOfMovement = new ArrayList<>();
        ArrayList<String> relativeMovement = new ArrayList<>();



//        if(cubePosition.equals("left")){ // old right side code
//            temp = drive.trajectoryBuilder(new Pose2d())
//                    .splineToLinearHeading(new Pose2d(17,-2, Math.toRadians(45)), Math.toRadians(45))
//                    .build();
//            lotsOfMovement.add(temp);
//            relativeMovement.add("drop first pixel");
//            temp = drive.trajectoryBuilder(new Pose2d(20,-5, Math.toRadians(55)))
//                    //TODO: Increase dist towards the canvas
//                    .splineToLinearHeading(new Pose2d(17,-36,Math.toRadians(-70)), Math.toRadians(-70))
//                    .build();
//            // armature should move down after this
//            relativeMovement.add("drop second pixel");
//            lotsOfMovement.add(temp);
//        }
//
//        if (cubePosition.equals("center")) {
//
//            temp = drive.trajectoryBuilder(new Pose2d())
//                    .splineToLinearHeading(new Pose2d(22,0, Math.toRadians(0)), Math.toRadians(0))
//                    .build();
//            relativeMovement.add("drop first pixel");
//            lotsOfMovement.add(temp);
//            temp = drive.trajectoryBuilder(new Pose2d(22,0, Math.toRadians(0)))
//                    .lineTo(new Vector2d(15,0))
//                    .build();
//            // armature should move down after this
//            relativeMovement.add("");
//            lotsOfMovement.add(temp);
//
//            temp = drive.trajectoryBuilder(new Pose2d(15,0), Math.toRadians(0))
//                    .splineToLinearHeading(new Pose2d(25,-34,Math.toRadians(-70)), Math.toRadians(-70))
//                    .build();
//            relativeMovement.add("drop second pixel");
//            lotsOfMovement.add(temp);
//        }
//
//        if (cubePosition.equals("right")) { // old left side code
//
//            temp = drive.trajectoryBuilder(new Pose2d())
//                    .splineToLinearHeading(new Pose2d(15,-7, Math.toRadians(-10)), Math.toRadians(-10))
//                    .build();
//            relativeMovement.add("drop first pixel");
//            lotsOfMovement.add(temp);
//            temp = drive.trajectoryBuilder(new Pose2d(15,-7, Math.toRadians(-10)))
//                    .splineToLinearHeading(new Pose2d(5,-7,Math.toRadians(1)), Math.toRadians(1))
//                    .build();
//            relativeMovement.add("");
//            lotsOfMovement.add(temp);
//            temp = drive.trajectoryBuilder(new Pose2d(5,-7, Math.toRadians(1)), Math.toRadians(1))
//                    .splineToLinearHeading(new Pose2d(30,-30,Math.toRadians(-70)), Math.toRadians(-70))
//                    .build();
//            relativeMovement.add("drop second pixel");
//            lotsOfMovement.add(temp);
//            // armature should move down after this
//        }

        TrajectorySequence untitled0 = null;

        if(cubePosition.equals("left")){ // old right side code
            untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-39.14, -65.23, Math.toRadians(90.00)))
                    .splineToSplineHeading(new Pose2d(-36.88, -33.07, Math.toRadians(180.00)), Math.toRadians(90.00))
                    .addTemporalMarker(() -> {
                        robot.startSpittingOutPixels();
                    })
                    .waitSeconds(1.5)
                    .addTemporalMarker(() -> {
                        robot.endSpittingOutPixels();
                    })
                    .build();
        }

        if(cubePosition.equals("center")){
            untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-39.14, -65.23, Math.toRadians(90.00)))
                    .splineToSplineHeading(new Pose2d(-36.88, -37.07, Math.toRadians(90.00)), Math.toRadians(90.00))
                    .addTemporalMarker(() -> {
                        robot.startSpittingOutPixels();
                    })
                    .waitSeconds(1.5)
                    .addTemporalMarker(() -> {
                        robot.endSpittingOutPixels();
                    })
                    .build();

        }

        if(cubePosition.equals("right")){
            untitled0 = drive.trajectorySequenceBuilder(new Pose2d(8.67, -65.23, Math.toRadians(90.82)))
                    .splineToSplineHeading(new Pose2d(11.07, -47.46, Math.toRadians(90.00)), Math.toRadians(90.00))
                    .splineToSplineHeading(new Pose2d(11.35, -31.38, Math.toRadians(0.00)), Math.toRadians(90.00))
                    .addTemporalMarker(() -> {
                        robot.startSpittingOutPixels();
                    })
                    .waitSeconds(1.5)
                    .addTemporalMarker(() -> {
                        robot.endSpittingOutPixels();
                    })
                    .build();

        }

        drive.setPoseEstimate(untitled0.start());
        drive.followTrajectorySequence(untitled0);

        //TrajectorySequence moveToBoard = drive.trajectorySequenceBuilder(untitled0.end());



        // Save more CPU resources when camera is no longer needed.


    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.

                //.setModelFileName("object_test1.tflite")

                .setModelFileName("red_cube_v1_model_20231026_113436.tflite")
                //setModelFileName("object_test2.tflite")


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
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

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
        tfod.setMinResultConfidence(0.3f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Function to add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        double highestConf = 0;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        Recognition recognition = null;
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for(int i = 0; i<currentRecognitions.size(); i++){
            if(currentRecognitions.get(i).getConfidence() > highestConf){
                recognition = currentRecognitions.get(i);
                highestConf = recognition.getConfidence();

            }
        }

        // Step through the list of recognitions and display info for each one.
        if(recognition != null){
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            if(0 < x && x < 400){
                telemetry.addData("Cube Pos: ", "left");
            }
            else if(500 < x && x < 900){
                telemetry.addData("Cube Pos: ", "center");
            }
            else if(1000 < x && x < 1280){
                telemetry.addData("Cube Pos: ", "right");
            }

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }


    }   // end method telemetryTfod()



}   // end class
