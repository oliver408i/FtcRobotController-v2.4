package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Basic program that does teleop. This is supposed to be a function-based,
 * modular program.
 * Ok it was SUPPOSED TO BE but not is just a hot mess of crappy AF code from Rian
 */

@Config
@TeleOp(name = "Omni Op Test Platform", group= "Linear Opmode")
public class TeleOpDynamicPlatform extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    double additionalYaw = 0;
    double leftYawCoolDown = runtime.seconds();
    double rightYawCoolDown = runtime.seconds();
    private CRServo spinny;
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



    boolean isHandedOff = false;
    //TODO: handoff if plate wants it

    boolean viperSlideAlternativeControl = false;

    Thread vsController = new Thread() {
        public void run() {
            // Init vars
            double viperSlideTarget = 0;
            double rotationsNeeded = 0;
            double encoderDrivingTarget = 0;
            double encoderDrivingTarget2 = 0;

            // Init viperslides
            robot.ViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.ViperSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.ViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.ViperSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.ViperSlide.setTargetPosition(0);
            robot.ViperSlide2.setTargetPosition(0);


            robot.ViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ViperSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.ViperSlide.setPower(0.5);
            robot.ViperSlide2.setPower(0.5);

            while (opModeIsActive()) {

                // Gamepad 2 x to activate alt control
                if (!viperSlideAlternativeControl && gamepad2.x) {
                    robot.ViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.ViperSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.ViperSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.ViperSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.ViperSlide.setPower(0);
                    robot.ViperSlide2.setPower(0);
                    viperSlideAlternativeControl = true;
                }

                if (!viperSlideAlternativeControl) {
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

                    robot.ViperSlide.setPower(0.5);
                    robot.ViperSlide2.setPower(0.5);

                    robot.ViperSlide.setTargetPosition((int) encoderDrivingTarget);
                    robot.ViperSlide2.setTargetPosition((int) encoderDrivingTarget2);

                    robot.ViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.ViperSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //viperslide1 is positive when extended
                    //viperslide2 is negative when extended

                } else {
                    robot.ViperSlide.setPower(gamepad2.left_stick_y*0.5);
                    robot.ViperSlide2.setPower(gamepad2.left_stick_y*0.5);
                }
            }
        }
    };

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
        robot.init(hardwareMap);
        runtime.reset();
        //starts after initialization (press start)

        spinny = hardwareMap.crservo.get("servo1");



        vsController.start();
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

            double max; // get top wheel speed

            double regularSpeed = 0.5;
            double superSpeed = 1;
            //superspeed is used only when there's a stretch
            // of ground to be covered, normally not
            // to be used during matches

            double axial   = -gamepad1.left_stick_y * regularSpeed;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * regularSpeed;
            double yaw     =  gamepad1.right_stick_x* regularSpeed;

            if(gamepad1.left_bumper){
                axial   *=  superSpeed/regularSpeed; //recalculating all values, least year's method was less elegant
                lateral *=  superSpeed/regularSpeed;
                yaw     *=  superSpeed/regularSpeed;
            }

            double leftFrontPower  = (axial + lateral + yaw);
            double rightFrontPower = (axial - lateral - yaw);
            double leftBackPower   = (axial - lateral + yaw);
            double rightBackPower  = (axial + lateral - yaw);
            // movement algorithm

            double right_trig = gamepad1.right_trigger;
            double left_trig  = gamepad1.left_trigger;
            // for strafing

            double avgMotorPower = (leftBackPower+leftFrontPower+rightBackPower+rightFrontPower)/4;
            // additionalyaw resource. we need to turn proportional to average speed of bot, and while this isn't perfect it works


            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            // calculate top wheel speed

            if (gamepad1.dpad_left && (runtime.seconds()-leftYawCoolDown)>1){
                additionalYaw-=0.01;
                leftYawCoolDown = runtime.seconds();
            }
            // need a cooldown for additionalyaw or it'll keep adding

            if (gamepad1.dpad_right && (runtime.seconds()-rightYawCoolDown)>1){
                additionalYaw+=0.01;
                rightYawCoolDown = runtime.seconds();
            }

            if (gamepad1.dpad_down) {

            }
/*
            if(gamepad2.dpad_up){
                robot.singleMotorEncoderMovements(telemetry,2,0.05,robot.EncoderTest)\
            }
 */



//            robot.ViperSlide.setPower(gamepad2.left_stick_y);
//            robot.ViperSlide2.setPower(gamepad2.left_stick_y);
            robot.linearActuator.setPower(gamepad2.right_stick_y);


            // intake code
            if(gamepad2.a){
                spinny.setPower(-0.6);
            }

            else if (gamepad2.b){
                spinny.setPower(0.6);
            }

            else{
                spinny.setPower(0);
            }

            // plane code
            if(gamepad2.dpad_up){
                robot.servo2.setPosition(0.25);
            }

            if(gamepad2.dpad_down){
                robot.servo2.setPosition(0);
            }

            // intake lifter code

            if(gamepad2.dpad_left){
                robot.spaghettiIntake.setPower(0.75); //shoot out
            }

            else if(gamepad2.dpad_right){
                robot.spaghettiIntake.setPower(-0.75); //suck in
            }

            else{
                robot.spaghettiIntake.setPower(0);
            }
            // swap negatives if rotating in unfavorable directions. change power as needed



            totalGamepad2TriggerInput = -gamepad2.left_trigger+gamepad2.right_trigger;

            robot.servo3.setPower(totalGamepad2TriggerInput);


            if(gamepad1.right_trigger > 0){
                leftFrontPower = right_trig;
                leftBackPower = -right_trig;
                rightFrontPower = -right_trig;
                rightBackPower = right_trig;
            }

            if(gamepad1.left_trigger > 0){
                leftFrontPower = -left_trig;
                leftBackPower = left_trig;
                rightFrontPower = left_trig;
                rightBackPower = -left_trig;
            }

            telemetry.addData("vector to degree test: ",robot.vectorToDegrees(axial,lateral));
            // i believe this is calculating angle of the robot relative to starting point

            if (max > 1) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            leftBackPower += additionalYaw*avgMotorPower;
            leftFrontPower += additionalYaw*avgMotorPower;
            rightBackPower -= additionalYaw*avgMotorPower;
            rightFrontPower -= additionalYaw*avgMotorPower;

            robot.leftFront.setPower(leftFrontPower*superSpeed);
            robot.rightFront.setPower(rightFrontPower*superSpeed);
            robot.leftBack.setPower(leftBackPower*superSpeed);
            robot.rightBack.setPower(rightBackPower*superSpeed);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.addData("Left Trigger:" , gamepad1.left_trigger);
//            telemetry.addData("Right Trigger", gamepad1.right_trigger);
//            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
//            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
//            telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
//            telemetry.addData("Right Stick X", gamepad1.right_stick_x);
            telemetry.addData("Additional Yaw: ", additionalYaw);
//            telemetry.addData("Axial: ", axial);
//            telemetry.addData("Lateral: ", lateral);
//            telemetry.addData("Yaw: ",yaw);
//            telemetry.addData("Servo Pos: ", robot.servo1.getController().getServoPosition(robot.servo1.getPortNumber()));
//            telemetry.addData("Servo Reported Power: ", robot.servo1.getPower());
            telemetry.addData("Viper Slide 1 Tgt: ", robot.ViperSlide.getTargetPosition());
            telemetry.addData("Viper Slide 2 Tgt: ", robot.ViperSlide2.getTargetPosition());
            telemetry.addData("Viper Slide 1 Curr: ", robot.ViperSlide.getCurrentPosition());
            telemetry.addData("Viper Slide 2 Curr: ", robot.ViperSlide2.getCurrentPosition());

            if (viperSlideAlternativeControl) {
                telemetry.addData("Viper Slide Control Mode","🆘VS ALT EXTEND MODE");
            } else {
                telemetry.addData("Viper Slide Control Mode","✳️Normal Target Mode");
            }

            telemetry.update();
        }
    }
}
