package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.graphics.Color;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Field;

/**
 * A basic color and touch sensor testing teleop
 *
 * For the future: color sensor is unreliable, may be scrapped in the future.
 * Undecided, will need further testing
 *
 * TODO: Add LED driver and code to this SAME teleop once hardware arrives
 */
@TeleOp
public class sensorsTest extends LinearOpMode{
    TouchSensor touch;
    ColorSensor color;
    DistanceSensor dist;
    String colorR;
    RevBlinkinLedDriver ledDriver;
    boolean prevTouch = false;
    Field[] ledPatternFields = RevBlinkinLedDriver.BlinkinPattern.class.getFields(); // Get all variables of the class
    int fIndex = 0;

    // This was a signed commit

    @Override
    public void runOpMode() {
        touch = hardwareMap.get(TouchSensor.class, "touch");
        color = hardwareMap.get(ColorSensor.class, "color");
        dist = hardwareMap.get(DistanceSensor.class, "color");


        // Remove if no led yet
        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledDriver");


        float hsvValues[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;
        waitForStart();
        //led.setPosition(0.5);
        while (opModeIsActive()) {

            if (touch.isPressed() && !prevTouch) {
                prevTouch = true;
                if (fIndex == ledPatternFields.length-1) {
                    fIndex = 0;
                } else {
                    fIndex++;
                }
            } else if (!touch.isPressed() && prevTouch) {
                prevTouch = false;
            }

            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(ledPatternFields[fIndex].getName()));


            // Grab color RGBs
            int R = (int) color.red();
            int G = (int) color.green();
            int B = (int) color.blue();

            /*
            Find the color ratios.

            The reason behind this and not using the raw color int values is
            the color sensor will output an integer more than 255. It is unknown to what scale this is, so instead we are using ratios
            The ratios (proportions) seem to be more stable at a set distance than the raw values

            Note that the pixel still passes through too fast for an accurate color detection
            */
            double R2G = (double)R / G;
            double G2B = (double)G / B;
            double R2B = (double)R / B;

            telemetry.addData("R2G", R2G);
            telemetry.addData("G2B", G2B);
            telemetry.addData("R2B", R2B);

            /*
            The ratio G:B seems to be most unique amongst the pixel colors
            Therefore, we are using it
            */
            double xe = G2B;
            if (0 <= xe && xe < 0.8) {
                colorR = "purple";
            } else if (0.8 <= xe && xe< 2) {
                colorR = "white";
            } else if (2 <= xe && xe < 3) {
                colorR = "green";
            } else if (3 <= xe && xe < 4) {
                colorR = "yellow";
            }

            telemetry.addData("color", colorR);

            /*
            HSV seems to be out of scale too
            Hue is correct, 0 - 999
            Saturation is correct, 0 - 1
            However, value goes extremely high
            */
            Color.RGBToHSV((int) (color.red() * SCALE_FACTOR),
                    (int) (color.green() * SCALE_FACTOR),
                    (int) (color.blue() * SCALE_FACTOR),
                    hsvValues);
            if (R > G && B > G && (R + B) > (2 * G)) {
                telemetry.addData("Purple", "Yes");
            }

            telemetry.addData("R", R);
            telemetry.addData("G", G);
            telemetry.addData("B", B);


            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Saturation", hsvValues[1]);
            telemetry.addData("Value", hsvValues[2]);

            telemetry.addData("ledPattern", ledPatternFields[fIndex].getName());
            telemetry.addData("touch", touch.getValue()); // Touch sensor direct output
            //telemetry.addData("Distance",dist.getDistance(DistanceUnit.CM));
            // May use touch.isPressed()
            telemetry.update();
        }
    }
}
