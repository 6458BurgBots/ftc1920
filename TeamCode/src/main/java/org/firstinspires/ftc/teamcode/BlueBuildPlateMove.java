package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Helper.BuildPlateServoHelper;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;
import org.firstinspires.ftc.teamcode.Helper.PickupArmHelper;

@Autonomous(name="BlueBuildPlateMove", group="Autonomous")
public class BlueBuildPlateMove extends OpMode{

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
        if (isInside){
            telemetry.addData("Ending Position", "inside");
        }
        else {
            telemetry.addData("Ending Position","outside");
        }
    }

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        buildPlateServoHelper = new BuildPlateServoHelper(telemetry, hardwareMap);
        buildPlateServoHelper.init();
        //sensorRange = hardwareMap.get(DistanceSensor.class, "range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        moveHelper.resetEncoders();
        moveHelper.runUsingEncoders();
        sensorColor = hardwareMap.get(ColorSensor.class, "colorsensor");
        buildPlateServoHelper.Close();

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
        telemetry.addData("state",state);
        telemetry.update();
        switch (state) {
            case 0:
                lastTime = getRuntime();
                state = 40;
                buildPlateServoHelper.Open();
                break;
            case 40: //Strafe left
                moveHelper.runMotorsToPosition(-600,600,-600,600);
                advanceToStateAfterTime(45,1);
                break;
            case 45:
                moveHelper.resetEncoders();
                state = 50;
                break;
            case 50: //Move backwards to square against wall
                moveHelper.encoderPowerLevel = .3;
                moveHelper.runMotorsToPosition(-400,-400,-400,-400);
                advanceToStateAfterTime(55,1);
                break;
            case 55:
                moveHelper.resetEncoders();
                state = 60;
                break;
            case 60: //Move forward towards building plate.
                moveHelper.encoderPowerLevel = .2;
                moveHelper.runMotorsToPosition(1600,1600,1600,1600);
                advanceToStateAfterTime(65,5);
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
                moveHelper.runMotorsToPosition(200, -200, -200, 200);
                advanceToStateAfterTime(68, 1);
                break;
            case 68:
                moveHelper.resetEncoders();
                state = 70;
                break;
            case 70: //Move back towards wall with building plate.
                moveHelper.encoderPowerLevel = .5;
                moveHelper.runMotorsToPosition(-1130, -1130, -1130, -1130);
                advanceToStateAfterTime(74, 2.5);
                break;
            case 74:
                moveHelper.resetEncoders();
                state = 78;
                break;
            case 78: //second Turn with building plate.
                moveHelper.encoderPowerLevel = .5;
                moveHelper.runMotorsToPosition(1000, -1000, -1000, 1000);
                advanceToStateAfterTime(79, 2);
                break;
         /*   case 65:
                moveHelper.resetEncoders();
                moveHelper.encoderPowerLevel = 1;
                state = 66;
                break;
            case 66:
                buildPlateServoHelper.Close();
                advanceToStateAfterTime(70,1);
                break;
            case 70: //Move back towards wall with building plate.
                moveHelper.encoderPowerLevel = .5;
                moveHelper.runMotorsToPosition(-1130,-1130,-1130,-1130);
                advanceToStateAfterTime(75, 2.5);
                break;
            case 75:
                moveHelper.encoderPowerLevel = 1;
                moveHelper.resetEncoders();
                state = 76;
                break;
            case 76: //Turn with building plate.
                moveHelper.encoderPowerLevel = .5;
                moveHelper.runMotorsToPosition(1100,-1100,-1100,1100);
                advanceToStateAfterTime(77, 2);
                break;
            case 77:
                moveHelper.encoderPowerLevel = 1;
                moveHelper.resetEncoders();
                state = 79;
                break; */
            case 79:
                buildPlateServoHelper.Open();
                advanceToStateAfterTime(80,1);
                break;
            case 80: //Turn with building plate.
                moveHelper.encoderPowerLevel = .5;
                moveHelper.runMotorsToPosition(750,750,750,750);
                advanceToStateAfterTime(85, 2);
                break;
            case 85:
                moveHelper.resetEncoders();
                state = 95;
                break;
            case 95:
                moveHelper.resetEncoders();
                //moveHelper.runWithoutEncoders();
                state = 120;
                break;


            case 115:
                //  moveHelper.resetEncoders();
                moveHelper.runUsingEncoders();
                state = 120;
                break;



            /* case 120:
                moveHelper.omniDrive(0,-.25,0);
                if (sensorColor.blue() > 30 && sensorColor.green() > 0 && sensorColor.red() > 0)
                {
                    double blueToGreen = (double)sensorColor.blue() / sensorColor.green();
                    double blueToRed = (double)sensorColor.blue() / sensorColor.red();
                    telemetry.addData("Red Ratio ", blueToRed);
                    telemetry.addData("Green Ratio  ", blueToGreen);

                    if (blueToGreen > 1.3 && blueToRed > 1.75)
                    {
                        state = 130;
                    }
                }
                break;
            case 130:
                moveHelper.omniDrive(0,0,0);
                break; */
            case 120://Strafe left/Right to get in line with end position.
                if (isInside){
                    moveHelper.runMotorsToPosition(900,-900,900,-900);
                }
                else {
                    moveHelper.runMotorsToPosition(-1000, 1000, -1000, 1000);
                }
                advanceToStateAfterTime(125, 1);
                break;
            case 125:
                moveHelper.resetEncoders();
                state = 130;
                break;
            case 130:
                moveHelper.runMotorsToPosition(600, 600,600,600);
                advanceToStateAfterTime(135, 2);
                break;
            case 135:
                moveHelper.resetEncoders();
                state = 140;
                break;
            case 140:
                moveHelper.runMotorsToPosition(-2000, -2000,-2000,-2000);
                advanceToStateAfterTime(150, 3);
                break;
            case 150:
                moveHelper.resetEncoders();
                break;

    }
/*        telemetry.addData("Red", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue", sensorColor.blue()); */
        telemetry.addData("State", state);
        telemetry.update();
    }
}
