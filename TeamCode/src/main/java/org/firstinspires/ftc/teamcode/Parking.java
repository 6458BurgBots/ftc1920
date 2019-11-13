package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@Autonomous(name= "Parking", group="Autonomous")
public class Parking extends OpMode {

    MoveHelper moveHelper;
    int state = 0;
    int strafeMultiplier = 1;
    boolean isRed;
    boolean isInside;
    double lastTime;
    ColorSensor sensorColor;

    public void init_loop() {
        if (gamepad1.a){ //outside
            isInside = false;
        }
        if (gamepad1.y) {//inside

            isInside = true;
        }
        if (gamepad1.b){
                isRed = true;
        }
        if (gamepad1.x){
                isRed = false;
        }
        if (isInside){
            telemetry.addData("Ending Position", "inside");
            state = 0;
        }
        else {
            telemetry.addData("Ending Position","outside");
            state = 16;
        }
        if (isRed){
            telemetry.addData("Alliance","red");
            strafeMultiplier = -1;
        }
        else {
            telemetry.addData("Alliance","blue");
            strafeMultiplier = 1;
        }

    }
    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        moveHelper.resetEncoders();
        moveHelper.runUsingEncoders();
        sensorColor = hardwareMap.get(ColorSensor.class, "colorsensor");
    }

    private void advanceToStateAfterTime(int newState, double duration) {

        if (getRuntime() > lastTime + duration) {
            lastTime = getRuntime();
            state = newState;
        }
    }

    @Override
    public void loop() {
        telemetry.addData("state", state);
        telemetry.update();
        switch (state) {
            case 0:
                lastTime = getRuntime();
                state = 10;
                break;
            case 10:
                moveHelper.runMotorsToPosition(-1700 * strafeMultiplier,1700 * strafeMultiplier, -1700 * strafeMultiplier, 1700 * strafeMultiplier);
                advanceToStateAfterTime(15, 2);
                break;
            case 15:
                moveHelper.resetEncoders();
                state = 16;
                break;
            case 16:
                //  moveHelper.resetEncoders();
                moveHelper.runUsingEncoders();
                state = 20;
                break;
            case 20:
                moveHelper.omniDrive(0,-.25,0);
                if (isRed) {
                    if (sensorColor.red() > 30 && sensorColor.green() > 0 && sensorColor.blue() > 0)
                    {
                        double redToGreen = (double)sensorColor.red() / sensorColor.green();
                        double redToBlue = (double)sensorColor.red() / sensorColor.blue();
                        telemetry.addData("Blue Ratio ", redToBlue);
                        telemetry.addData("Green Ratio  ", redToGreen);

                        if (redToGreen > 2 && redToBlue > 2)
                        {
                            state = 30;
                        }
                    }
                } else {
                    if (sensorColor.blue() > 30 && sensorColor.green() > 0 && sensorColor.red() > 0) {
                        double blueToGreen = (double) sensorColor.blue() / sensorColor.green();
                        double blueToRed = (double) sensorColor.blue() / sensorColor.red();
                        telemetry.addData("Red Ratio ", blueToRed);
                        telemetry.addData("Green Ratio  ", blueToGreen);

                        if (blueToGreen > 1.3 && blueToRed > 1.5) {
                            state = 30;
                        }
                    }
                }
                break;
            case 30:
                moveHelper.omniDrive(0,0,0);
                break;
        }

    }


}