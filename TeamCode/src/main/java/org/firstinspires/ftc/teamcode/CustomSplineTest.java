package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Custom Spline Test")
public class CustomSplineTest extends LinearOpMode{
    //RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        //robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(10,20), Math.toRadians(0))
                /*.splineToConstantHeading(new Vector2d(-10,20), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(10,-40), Math.toRadians(0))*/
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
    }

}
