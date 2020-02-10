package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Helper.BuildPlateServoHelper;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@Autonomous(name="RedBuildPlateMove", group="Autonomous")
public class RedBuildPlateMove extends OpMode {

    MoveHelper moveHelper;
    BuildPlateServoHelper buildPlateServoHelper;
    DistanceSensor sensorRange;
    int state = 0;
    double lastTime;
    ColorSensor sensorColor;
    public int long_move = -4700;
    boolean isInside;

/*
    public void init_loop() {
        if (gamepad1.y){
            long_move = -6250;
            telemetry.addData("Starting Position","outside");
        }
        if (gamepad1.a){
            long_move = -4700;
            telemetry.addData("Starting Position","inside");
        }
    }
*/

    public void init_loop() {
        if (gamepad1.a) { //outside
            isInside = false;
        }
        if (gamepad1.y) {//inside
            isInside = true;
        }
        if (isInside) {
            telemetry.addData("Ending Position", "inside");
        } else {
            telemetry.addData("Ending Position", "outside");
        }
    }

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        buildPlateServoHelper = new BuildPlateServoHelper(telemetry, hardwareMap);
        buildPlateServoHelper.init();
        //sensorRange = hardwareMap.get(DistanceSensor.class, "range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;
        moveHelper.resetEncoders();
        moveHelper.runUsingEncoders();
        sensorColor = hardwareMap.get(ColorSensor.class, "colorsensor");
        buildPlateServoHelper.Open();
    }

    // Noticed that each case was similar, so created a procedure called advancedToStateAfterTime
    // parameters include the newState, which refers to the new value being assigned to state at end of duration
    // and duration, which refers to the amount of time before moving to the new state
    private void advanceToStateAfterTime(int newState, double duration) {

        if (getRuntime() > lastTime + duration) {
            lastTime = getRuntime();
            state = newState;
        }
    }

    // Only go on after motors have stopped running
    private void advanceToStateAfterAtPosition(int newState) {
        if (!moveHelper.areMotorsBusy()) {
            moveHelper.resetEncoders();
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
                state = 40;
                break;
            case 40: //Strafe right
                moveHelper.encoderPowerLevel = .5;
                moveHelper.runMotorsToPosition(450, -450, 450, -450);
                advanceToStateAfterTime(45, 1);
                break;
            case 45:
                moveHelper.resetEncoders();
                state = 50;
                break;
            case 50: //Move backwards to square against wall
                moveHelper.encoderPowerLevel = .3;
                moveHelper.runMotorsToPosition(-600, -600, -600, -600);
                advanceToStateAfterTime(55, 1);
                break;
            case 55:
                moveHelper.resetEncoders();
                state = 60;
                break;
            case 60: //Move forward towards building plate.
                moveHelper.encoderPowerLevel = .3;
                moveHelper.runMotorsToPosition(1600, 1600, 1600, 1600);
                advanceToStateAfterTime(65, 5);
                break;
            case 65:
                moveHelper.resetEncoders();
                moveHelper.encoderPowerLevel = 1;
                state = 66;
                break;
            case 66:
                buildPlateServoHelper.Close();
                advanceToStateAfterTime(67, 1);
                break;
            case 67: //first Turn with building plate.
                moveHelper.encoderPowerLevel = .5;
                moveHelper.runMotorsToPosition(-200, 200, 200, -200);
                advanceToStateAfterTime(68, 1);
                break;
            case 68:
                moveHelper.resetEncoders();
                state = 70;
                break;
            case 70: //Move back towards wall with building plate.
                moveHelper.encoderPowerLevel = .5;
                moveHelper.runMotorsToPosition(-1130, -1130, -1130, -1130);
                advanceToStateAfterTime(80, 2.5);
                break;
            case 80:
                moveHelper.resetEncoders();
                state = 90;
                break;
            case 90: //second Turn with building plate.
                moveHelper.encoderPowerLevel = .5;
                moveHelper.runMotorsToPosition(-1000, 1000, 1000, -1000);
                advanceToStateAfterTime(100, 1);
                break;
            case 100:
                moveHelper.encoderPowerLevel = 1;
                moveHelper.resetEncoders();
                state = 110;
                break;
            case 110:
                buildPlateServoHelper.Open();
                advanceToStateAfterTime(120, 1);
                break;
            case 120:
                moveHelper.encoderPowerLevel = .5;
                moveHelper.runMotorsToPosition(400, 400, 400, 400);
                advanceToStateAfterTime(130, 2);
                break;
            case 130:
                moveHelper.resetEncoders();
                state = 140;
                break;
            case 140:
                moveHelper.resetEncoders();
                //moveHelper.runWithoutEncoders();
                state = 150;
                break;


            case 150://Strafe left/Right to get in line with end position.
                if (isInside) {
                    moveHelper.runMotorsToPosition(-1000, 1000, -1000, 1000);
                } else {
                    moveHelper.runMotorsToPosition(1000, -1000, 1000, -1000);
                }
                advanceToStateAfterTime(160, 1);
                break;
            case 160:
                moveHelper.resetEncoders();
                state = 170;
                break;
            case 170:
                moveHelper.runMotorsToPosition(400, 400, 400, 400);
                advanceToStateAfterTime(180, 1);
            case 180:
                moveHelper.resetEncoders();
                state = 190;
                break;
            case 190:
                moveHelper.runMotorsToPosition(-2000, -2000, -2000, -2000);
                advanceToStateAfterTime(200, 3);
                break;
            case 200:
                moveHelper.resetEncoders();
                break;

                /*
 case 120:
                moveHelper.omniDrive(0,-.25,0);
                if (sensorColor.blue() > 30 && sensorColor.green() > 0 && sensorColor.red() > 0)
                {
                    double redToGreen = (double)sensorColor.red() / sensorColor.green();
                    double redToBlue = (double)sensorColor.blue() / sensorColor.red();
                    telemetry.addData("Blue Ratio ", redToBlue);
                    telemetry.addData("Green Ratio  ", redToGreen);

                    if (redToGreen > 1.3 && redToBlue > 1.75)
                    {
                        state = 130;
                    }
                }
                break;
            case 130:
                moveHelper.omniDrive(0, 0, 0);
                break;*/
        }
        /*telemetry.addData("Red", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue", sensorColor.blue());*/
        telemetry.addData("State", state);
        telemetry.update();
    }
}

