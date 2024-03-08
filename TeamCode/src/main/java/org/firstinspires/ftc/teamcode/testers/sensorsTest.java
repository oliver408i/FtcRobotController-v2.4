package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.graphics.Color;


@TeleOp
public class sensorsTest extends LinearOpMode{
    TouchSensor touch;
    ColorSensor color;
    String colorR;

    @Override
    public void runOpMode() {
        touch = hardwareMap.get(TouchSensor.class, "touch");
        color = hardwareMap.get(ColorSensor.class, "color");
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        waitForStart();
        while (opModeIsActive()) {

            int R = (int) color.red();
            int G = (int) color.green();
            int B = (int) color.blue();
            double R2G = (double)R / G;
            double G2B = (double)G / B;
            double R2B = (double)R / B;

            telemetry.addData("R2G", R2G);
            telemetry.addData("G2B", G2B);
            telemetry.addData("R2B", R2B);

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
            telemetry.addData("touch", touch.getValue());
            telemetry.update();
        }
    }
}
