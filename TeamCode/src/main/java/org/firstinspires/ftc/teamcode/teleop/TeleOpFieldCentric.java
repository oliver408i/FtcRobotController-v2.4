package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

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

        //
        CRServo spinny = hardwareMap.crservo.get("servo1");

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(new Pose2d(0,0,0));

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y*0.5,
                    -gamepad1.left_stick_x*0.5
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