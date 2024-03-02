package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This teleop program aims to create a separate teleop drive that is field-centric.
 * The drive difference from the normal teleop, but the attachement controls are the same
 * [To be implemented™]
 */
@TeleOp(group = "advanced")
public class TeleOpFieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor ViperSlide = hardwareMap.get(DcMotor.class, "ViperSlide");
        DcMotor ViperSlide2 = hardwareMap.get(DcMotor.class, "ViperSlide2");
        DcMotor spaghettiIntake = hardwareMap.get(DcMotor.class, "spaghettiIntake");

        double speedMultiplier = 0.5;

        CRServo spinny = hardwareMap.get(CRServo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");

        ViperSlide.setDirection(DcMotor.Direction.FORWARD);
        ViperSlide2.setDirection(DcMotor.Direction.FORWARD);
        spaghettiIntake.setDirection(DcMotor.Direction.FORWARD);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(new Pose2d(0,0,0));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        Thread vsController = new Thread() {
            double encoderDrivingTarget = 0;
            boolean viperSlideAlternativeControl = false;

            double encoderDrivingTarget2 = 0;
            public void run() {
                // Init vars
                double viperSlideTarget = 0;
                double rotationsNeeded = 0;
                // Init viperslides
                ViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ViperSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                ViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ViperSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                ViperSlide.setTargetPosition(0);
                ViperSlide2.setTargetPosition(0);


                ViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ViperSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                ViperSlide.setPower(0.5);
                ViperSlide2.setPower(0.5);

                while (opModeIsActive()) {

                    // Gamepad 2 x to activate alt control
                    if (!viperSlideAlternativeControl && gamepad2.x) {
                        ViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        ViperSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        ViperSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        ViperSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        ViperSlide.setPower(0);
                        ViperSlide2.setPower(0);
                        viperSlideAlternativeControl = true;
                    }

                    if (viperSlideAlternativeControl && gamepad2.y) {
                        // This ensures the slide is in 0 position
                        ViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        ViperSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        ViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        ViperSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        ViperSlide.setTargetPosition(0);
                        ViperSlide2.setTargetPosition(0);


                        ViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ViperSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        ViperSlide.setPower(0.5);
                        ViperSlide2.setPower(0.5);
                        viperSlideTarget = 0;
                        rotationsNeeded = 0;
                        this.encoderDrivingTarget = 0;
                        this.encoderDrivingTarget2 = 0;
                        viperSlideAlternativeControl = false;
                    }

                    if (!viperSlideAlternativeControl) {
                        ViperSlide.setPower(0.5);
                        ViperSlide2.setPower(0.5);

                        if (gamepad2.left_stick_y < 0.1 && gamepad2.left_stick_y > -0.1) {
                            ViperSlide.setTargetPosition((int) ViperSlide.getCurrentPosition());
                            ViperSlide2.setTargetPosition((int) ViperSlide2.getCurrentPosition());
                            this.encoderDrivingTarget = ViperSlide.getTargetPosition();
                            double rotationsNeed = encoderDrivingTarget / RobotHardware.TICK_COUNT;
                            viperSlideTarget = rotationsNeed * RobotHardware.VS_CIRCUMFERENCE;

                        } else {
                            viperSlideTarget += gamepad2.left_stick_y * 0.5;
                            rotationsNeeded = viperSlideTarget / RobotHardware.VS_CIRCUMFERENCE;

                            encoderDrivingTarget = rotationsNeeded * RobotHardware.TICK_COUNT;

                            if (encoderDrivingTarget > 1) {
                                encoderDrivingTarget = 0;
                            }

                            if (encoderDrivingTarget < -3501) {
                                encoderDrivingTarget = -3500;
                            }

                            encoderDrivingTarget2 = -encoderDrivingTarget;



                            ViperSlide.setTargetPosition((int) encoderDrivingTarget);
                            ViperSlide2.setTargetPosition((int) encoderDrivingTarget2);
                        }

                        ViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ViperSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        //viperslide1 is positive when extended
                        //viperslide2 is negative when extended

                    } else {
                        ViperSlide.setPower(gamepad2.left_stick_y*0.5);
                        ViperSlide2.setPower(gamepad2.left_stick_y*-0.5);
                    }
                }
            }
        };

        
        
        waitForStart();
        if (isStopRequested()) return;
        
        vsController.start();
        
        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y*speedMultiplier,
                    -gamepad1.left_stick_x*speedMultiplier
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x*0.5
                    )
            );
            // Spinny (Intake) control
            // A - pixel in
            if(gamepad2.a){
                spinny.setPower(-0.6);
            }

            //B - pixel out
            else if (gamepad2.b) {
                spinny.setPower(0.6);
            }
            else{
                spinny.setPower(0);
            }

            //plane launcher code
            if(gamepad2.dpad_down){ //depends on servo orientation. swap at will
                servo2.setPosition(0.25);
            }

            if(gamepad2.dpad_up){
                servo2.setPosition(0);
            }

            // superspeed
            if (gamepad1.left_bumper) {
                speedMultiplier = 1;
            } else {
                speedMultiplier = 0.5;
            }

            // intake lifter code
            if(gamepad2.dpad_left){
                spaghettiIntake.setPower(0.75); //shoot out
            }else if(gamepad2.dpad_right){
                spaghettiIntake.setPower(-0.75); //suck in
            }else{
                spaghettiIntake.setPower(0);
            }

            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}