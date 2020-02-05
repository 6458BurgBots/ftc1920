package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.BlockArmServoHelper;
import org.firstinspires.ftc.teamcode.Helper.IMUHelper;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@Autonomous(name="DepotSamplingBlue", group="Autonomous")
public class DepotSamplingBlue extends OpMode {

    MoveHelper moveHelper;
    DistanceSensor sensorRange;
    ColorSensor sensorColor;
    ColorSensor blockColor;
    BlockArmServoHelper blockArmServoHelper;
    IMUHelper imuHelper;
    double lastTime;
    int state = 0;
    int firstBlockDistance;
    private static int BRIDGE_TRAVEL_DISTANCE = 1950;
    private static int SKYSTONE_DISTANCE = 1100;
    private int returndist = firstBlockDistance - BRIDGE_TRAVEL_DISTANCE - SKYSTONE_DISTANCE;
    private static double SLOW_SPEED = .2;
    private static double NORMAL_SPEED = .7;
    private static double FAST_SPEED = 1;

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        moveHelper.resetEncoders();
        moveHelper.runUsingEncoders();
        sensorColor = hardwareMap.get(ColorSensor.class, "colorsensor");
        blockColor = hardwareMap.get(ColorSensor.class,"blockdetector");
        blockArmServoHelper = new BlockArmServoHelper(telemetry, hardwareMap);
        blockArmServoHelper.init();
        imuHelper = new IMUHelper(telemetry, hardwareMap);
        imuHelper.moveHelper = moveHelper;
        imuHelper.init();
    }

    @Override
    public void init_loop() {
        imuHelper.init_loop();
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
                state = 20;
                blockArmServoHelper.Open();
                break;

            case 20:        //backwards move to line of blocks
                moveHelper.runMotorsToPosition(900,900,900,900);
                advanceToStateAfterTime(30, 1.5);
                break;
            case 30:
                moveHelper.resetEncoders();
                state = 40;
                break;
            case 40:        //left turn to line up for color sensing
                moveHelper.runMotorsToPosition(750,-750,-750,750);
                advanceToStateAfterTime(50,1);
                break;
            case 50:
                moveHelper.resetEncoders();
                state = 60;
                break;

            case 60: // Adjust angle so that we can move straight to the wall
                imuHelper.turnTo(-90);
                advanceToStateAfterTime(100, 1);
                break;

            case 100:        //move back to the wall
                moveHelper.encoderPowerLevel = SLOW_SPEED;
                moveHelper.runMotorsToPosition(600,-600,600,-600);
                if (isCloseToBlock(blockColor)){
                    lastTime = getRuntime();
                    state = 200;
                }
                advanceToStateAfterTime(997,3); //failsafe
                break;
            case 200:
                moveHelper.runWithoutEncoders();
                state = 205;
                break;

            case 205: // Adjust angle so that we can move straight to the wall
                imuHelper.turnTo(-90);
                advanceToStateAfterTime(207, 1);
                break;

            case 207:
                moveHelper.resetEncoders();
                moveHelper.runUsingEncoders();
                state = 210;
                break;

            case 210:        //light logic and move to sense
                moveHelper.runMotorsToPosition(-1600,-1600,-1600,-1600);
                if(isSkyStone(blockColor)){
                    firstBlockDistance = moveHelper.getEncoderValue();
                    lastTime = getRuntime();
                    returndist = firstBlockDistance - BRIDGE_TRAVEL_DISTANCE - SKYSTONE_DISTANCE;
                    state = 220;
                }
                advanceToStateAfterTime(998,5);
                break;
            case 220:
                moveHelper.resetEncoders();
                state = 230;
                break;

            case 230:        //lower block Arm
                blockArmServoHelper.Close();
                advanceToStateAfterTime(240,.5);
                break;

            case 240:        //Strafe to separate from the line
                moveHelper.encoderPowerLevel = NORMAL_SPEED;
                moveHelper.runMotorsToPosition(-600,600,-600,600);
                advanceToStateAfterTime(250,1);
                break;
            case 250:
                moveHelper.runWithoutEncoders(); // Turn off encoders so the next move can be a manual turn
                moveHelper.encoderPowerLevel = FAST_SPEED;
                state = 255;
                break;
            case 255: //Adjust angle so robot runs past the bridge
                imuHelper.turnTo(-92);
                advanceToStateAfterTime(257, 1.5);
                break;

            case 257:
                moveHelper.resetEncoders();
                moveHelper.runUsingEncoders();
                state = 260;
                break;

            case 260:        //run to pass the bridge
                moveHelper.encoderPowerLevel = NORMAL_SPEED;
                int dist = BRIDGE_TRAVEL_DISTANCE - firstBlockDistance;
                moveHelper.runMotorsToPosition(dist, dist, dist, dist);
                advanceToStateAfterTime(270,2.5);
                break;
            case 270:
                moveHelper.resetEncoders();
                moveHelper.runUsingEncoders();
                state = 280;
                break;

            case 280: //Block Arm up
                blockArmServoHelper.Open();
                moveHelper.runUsingEncoders();
                moveHelper.omniDrive(0,0,0);
                advanceToStateAfterTime(300,.5);
                break;
            case 300:
                int dist2 = BRIDGE_TRAVEL_DISTANCE - firstBlockDistance;
                moveHelper.runMotorsToPosition(returndist, returndist, returndist, returndist);
                advanceToStateAfterTime(305,3);
                break;
            case 305:
                moveHelper.omniDrive(0,0,0);
                moveHelper.resetEncoders();
                moveHelper.runWithoutEncoders();
                advanceToStateAfterTime(307,1);
                break;
            case 307: // Adjust angle so that we can move straight to the wall
                imuHelper.turnTo(-90);
                advanceToStateAfterTime(308, 1);
                break;
            case 308:
                advanceToStateAfterTime(310, 1);
                moveHelper.omniDrive(0,0,0);
                moveHelper.runUsingEncoders();
                moveHelper.encoderPowerLevel = SLOW_SPEED;
                break;
            case 310:
                moveHelper.encoderPowerLevel = SLOW_SPEED;
                moveHelper.runMotorsToPosition(900,-900,900,-900);
                if (isCloseToBlock(blockColor)){
                    lastTime = getRuntime();
                    state = 312;
                }
                advanceToStateAfterTime(997,3); //failsafe
                break;
            case 312:
                moveHelper.resetEncoders();
                moveHelper.runUsingEncoders();
                state = 315;
                break;
            case 315:        //light logic and move to sense
                moveHelper.runMotorsToPosition(-1600,-1600,-1600,-1600);
                if(isSkyStone(blockColor)){
                    firstBlockDistance = moveHelper.getEncoderValue();
                    lastTime = getRuntime();
                    returndist = firstBlockDistance - BRIDGE_TRAVEL_DISTANCE - SKYSTONE_DISTANCE;
                    state = 320;
                }
                advanceToStateAfterTime(998,5);
                break;
            case 320:        //lower block Arm
                moveHelper.resetEncoders();
                blockArmServoHelper.Close();
                advanceToStateAfterTime(325,1);
                break;
            case 325:
                moveHelper.resetEncoders();
                moveHelper.runUsingEncoders();
                state = 330;
                break;
            case 330:        //Strafe to separate from the line
                moveHelper.encoderPowerLevel = .7;
                moveHelper.runMotorsToPosition(-600,600,-600,600);
                advanceToStateAfterTime(335,3);
                break;
            case 335: //Adjust angle so robot runs past the bridge
                imuHelper.turnTo(-92);
                advanceToStateAfterTime(337, 1.5);
                break;
            case 337:
                moveHelper.resetEncoders();
                moveHelper.runUsingEncoders();
                state = 340;
                break;
            case 340:        //run to pass the bridge
                moveHelper.runMotorsToPosition(-returndist, -returndist, -returndist, -returndist);
                advanceToStateAfterTime(345,3);
                break;
            case 345:
                moveHelper.resetEncoders();
                state = 347;
                break;
            case 347: //Block Arm up
                blockArmServoHelper.Open();
                moveHelper.runUsingEncoders();
                moveHelper.omniDrive(0,0,0);
                advanceToStateAfterTime(350,.5);
                break;
            case 350:
                moveHelper.runMotorsToPosition(-600,-600,-600,-600);
                advanceToStateAfterTime(360,1);
                break;
          /*  case 300:
                moveHelper.omniDrive(0,-.25,0);
                if (sensorColor.blue() > 30 && sensorColor.green() > 0 && sensorColor.red() > 0) {
                    double blueToGreen = (double) sensorColor.blue() / sensorColor.green();
                    double blueToRed = (double) sensorColor.blue() / sensorColor.red();
                    telemetry.addData("Red Ratio ", blueToRed);
                    telemetry.addData("Green Ratio  ", blueToGreen);

                    if (blueToGreen > 1.2 && blueToRed > 1.5) {
                        state = 310;
                    }
                }
                advanceToStateAfterTime(310,3);
                break;*/
            case 360:
                moveHelper.omniDrive(0,0,0);
                state = 370;
                break;

            case 370:
                telemetry.addData("End Program", "");
                break;
            case 997:
                telemetry.addData("End: didn't sense stone line", "");
                moveHelper.omniDrive(0,0,0);
                break;
            case 998:
                telemetry.addData("End: never found skystone", "");
                moveHelper.omniDrive(0,0,0);
                break;
        }
    }

    private boolean isSkyStone(ColorSensor blockColor){
        double redToBlue = blockColor.red() / blockColor.blue();
        double greenToBlue = blockColor.green() / blockColor.blue();
        if (isCloseToBlock(blockColor)) {

            if (redToBlue < 2 && greenToBlue < 3) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

    private boolean isCloseToBlock(ColorSensor blockColor){
        double totalBlockSensorValues = blockColor.red() + blockColor.blue() + blockColor.green();
        if (totalBlockSensorValues < 1500){
            return false;
        }
        return true;
    }
}