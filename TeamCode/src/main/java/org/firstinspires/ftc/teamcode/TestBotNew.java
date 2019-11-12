package org.firstinspires.ftc.teamcode;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@TeleOp(name="TestBotOld", group="TeleOp")
public class TestBotNew extends OpMode{

    public static double SAMPLE_SERVO_CLOSED = 1;
    public static double SAMPLE_SERVO_OPEN = .5;
    MoveHelper moveHelper;
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
    protected Servo plateArmServo;


    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
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
        plateArmServo = hardwareMap.servo.get("platearm");
        plateArmServo.setPosition(SAMPLE_SERVO_OPEN);

    }

    @Override
    public void loop() {
        double shortRange = 4;
        double longRange = 6;
        //double range = sensorRange.getDistance(DistanceUnit.INCH);
        moveHelper.checkTeleOp(gamepad1,gamepad2);

        if (gamepad1.right_trigger > 0) {
            telemetry.addData("plateArm:", "CLOSED");
           plateArmServo.setPosition(SAMPLE_SERVO_CLOSED);
        }
        if (gamepad1.left_trigger > 0) {
            telemetry.addData("plateArm:", "OPENED");
            plateArmServo.setPosition(SAMPLE_SERVO_OPEN);
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
    public void open(){
        plateArmServo.setPosition(SAMPLE_SERVO_OPEN);
    }

    public void close(){
        plateArmServo.setPosition(SAMPLE_SERVO_CLOSED);
    }
}
