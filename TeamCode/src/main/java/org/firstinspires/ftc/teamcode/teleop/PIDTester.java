package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Basic program that does teleop. This is supposed to be a function-based,
 * modular program.
 */

@Config
@Disabled
@TeleOp(name = "PID Tester", group= "Concept")
public class PIDTester extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    double additionalYaw = 0;
    double leftYawCoolDown = runtime.seconds();
    double rightYawCoolDown = runtime.seconds();
    double servoPos = 0;

    double totalGamepad2TriggerInput = 0;



//    private PIDController controller;
//    public static double p=0,i=0,d=0;
//    public static double f=0;
//
//    public static int target1 = 0;
//    public static int target2 = 0;
//    private final double ticks_in_degree = RobotHardware.TICK_COUNT/360;
//    private DcMotorEx slideMotor1;
//    private DcMotorEx slideMotor2;
//
//    double pid;
//    double pid2;
//    double ff1;
//    double ff2;
//    double slidePower1;
//    double slidePower2;

    double viperSlideTarget = 0;
    double rotationsNeeded = 0;
    double encoderDrivingTarget = 0;
    double encoderDrivingTarget2 = 0;

    boolean isHandedOff = false;
    //TODO: handoff if plate wants it

    DcMotor ViperSlide;
    DcMotor ViperSlide2;


    @Override
    public void runOpMode() {

//        controller = new PIDController(p,i,d);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        slideMotor1 = hardwareMap.get(DcMotorEx.class, "ViperSlide");
//        slideMotor2 = hardwareMap.get(DcMotorEx.class, "ViperSlide2");
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
        // initalizes

        waitForStart();
        ViperSlide = hardwareMap.get(DcMotor.class, "ViperSlide");
        ViperSlide2 = hardwareMap.get(DcMotor.class, "ViperSlide2");
        runtime.reset();

        ViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ViperSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ViperSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ViperSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // run until stop is pressed
        while (opModeIsActive()) {
//            controller.setPID(p,i,d);
//
//            pid = controller.calculate(slideMotor1.getCurrentPosition(), target1);
//            pid2 = controller.calculate(slideMotor2.getCurrentPosition(), target2);
//
//            ff1 = Math.cos(Math.toRadians(target1/ticks_in_degree))*f;
//            ff2 = Math.sin(Math.toRadians(target2/ticks_in_degree))*f;
//
//            slidePower1 = pid + ff1;
//            slidePower2 = pid2 + ff2;
//
//            slideMotor1.setPower(slidePower1);
//            slideMotor2.setPower(slidePower2);



            viperSlideTarget += -gamepad2.left_stick_y*0.5;
            rotationsNeeded = viperSlideTarget/RobotHardware.VS_CIRCUMFERENCE;
            encoderDrivingTarget = rotationsNeeded*RobotHardware.TICK_COUNT;

            if(encoderDrivingTarget < 0){
                encoderDrivingTarget = 0;
            }

            if(encoderDrivingTarget > 4500){
                encoderDrivingTarget = 4500;
            }

            encoderDrivingTarget2 = encoderDrivingTarget*-1;

            ViperSlide.setPower(0.5);
            ViperSlide2.setPower(0.5);

            ViperSlide.setTargetPosition((int) encoderDrivingTarget);
            ViperSlide2.setTargetPosition((int) encoderDrivingTarget*-1);

            ViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ViperSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            telemetry.addData("Viper Slide 1 Tgt: ", ViperSlide.getTargetPosition());
            telemetry.addData("Viper Slide 2 Tgt: ", ViperSlide2.getTargetPosition());
            telemetry.addData("Viper Slide 1 Curr: ", ViperSlide.getCurrentPosition());
            telemetry.addData("Viper Slide 2 Curr: ", ViperSlide2.getCurrentPosition());
            telemetry.update();
        }
    }
}
