//Old

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.BuildPlateServoHelper;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@TeleOp(name="TeleOpOld", group="TeleOp")
public class TeleOpOld extends OpMode{

    public static double SAMPLE_SERVO_CLOSED = 1;
    public static double SAMPLE_SERVO_OPEN = .5;

    MoveHelper moveHelper;
    BuildPlateServoHelper buildPlateServoHelper;
    //DistanceSensor sensorRange;
    //ColorSensor sensorColor;
    //float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    //final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
    int relativeLayoutId;
    View relativeLayout;

    public void init_loop() {
        if (moveHelper == null){
            moveHelper = new MoveHelper(telemetry, hardwareMap);
        }
        if (gamepad1.a){ // Left stick is cardinal moves, rotation on right
            moveHelper.joystickJacob = true;
            telemetry.addData("Joystick setup", " Jacob's Way");
        }
        if (gamepad1.y) { // Left stick is rotation, cardinal moves on right
            moveHelper.joystickJacob = false;
            telemetry.addData("Joystick setup", " Thomas' Way");
        }
    }

    @Override
    public void init() {
        if (moveHelper == null){
            moveHelper = new MoveHelper(telemetry, hardwareMap);
        }
        moveHelper.init();
        buildPlateServoHelper = new BuildPlateServoHelper(telemetry, hardwareMap);
        buildPlateServoHelper.init();

        //sensorRange = hardwareMap.get(DistanceSensor.class, "range");
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        moveHelper.resetEncoders();
        //moveHelper.runUsingEncoders();
        moveHelper.runWithoutEncoders();
        //sensorColor = hardwareMap.get(ColorSensor.class, "colorsensor");
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        //relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        //relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    }

    @Override
    public void loop() {
        double shortRange = 4;
        double longRange = 6;
        //double range = sensorRange.getDistance(DistanceUnit.INCH);
        moveHelper.checkTeleOp(gamepad1,gamepad2);

        if (gamepad1.right_trigger > 0) {
            telemetry.addData("plateArm:", "CLOSED");
            buildPlateServoHelper.Close();
        }
        if (gamepad1.left_trigger > 0) {
            telemetry.addData("plateArm:", "OPENED");
            buildPlateServoHelper.Open();
        }
 /*       if (range > longRange){                       Laser Rangefinder Wallfollow Logic
            moveHelper.rturn(1);
        }
        else if (range < shortRange){
            moveHelper.lturn(1);
        }
        else {
            moveHelper.checkTeleOp(gamepad1,gamepad2);
        }

  */
/*        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        // send the info back to driver station using telemetry function.
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("Hue", hsvValues[0]);


        //moveHelper.showEncoderValues();
        telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.update();
        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
*/
        if (gamepad1.left_stick_button){        //Resets Encoder Values With Left Stick Button
            moveHelper.resetEncoders();
            moveHelper.runUsingEncoders();
        }
        if (gamepad1.right_stick_button){
            moveHelper.runWithoutEncoders();
        }
    }
}
