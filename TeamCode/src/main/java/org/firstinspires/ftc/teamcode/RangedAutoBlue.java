package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.BlockArmServoHelper;
import org.firstinspires.ftc.teamcode.Helper.IMUHelper;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@Autonomous(name="RangedAutoBlue", group="Autonomous")
public class RangedAutoBlue extends OpMode {

    MoveHelper moveHelper;
    DistanceSensor sensorRange;
    ColorSensor sensorColor;
    ColorSensor blockColor;
    BlockArmServoHelper blockArmServoHelper;
    IMUHelper imuHelper;
    double lastTime;
    int state = 0;
    int firstBlockDistance;
    int secondBlockDistance;
    private static int BRIDGE_TRAVEL_DISTANCE = 1950;
    private static int SKYSTONE_DISTANCE = 900;     //Here!!! Was 1200 #2
    private int returndist;
    private static double SLOW_SPEED = .2;
    private static double NORMAL_SPEED = .7;
    private static double FAST_SPEED = 1;
    private static double NINETY_IN_RADIANS = Math.PI/2;

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        moveHelper.resetEncoders();
        moveHelper.runUsingEncoders();
        sensorColor = hardwareMap.get(ColorSensor.class, "colorsensor");
        blockColor = hardwareMap.get(ColorSensor.class,"blockdetector");
        blockArmServoHelper = new BlockArmServoHelper(telemetry, hardwareMap);
        sensorRange = hardwareMap.get(DistanceSensor.class, "range");
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
        switch (state) {
            case 0:
                lastTime = getRuntime();
                state = 20;
                blockArmServoHelper.Open();
                break;

            case 20:        //backwards move to line of blocks
                moveHelper.runMotorsToPosition(900,900,900,900);
                advanceToStateAfterTime(30, 1.25);
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
                advanceToStateAfterTime(70, .75);
                break;
            case 70:
                moveHelper.resetEncoders();
                state = 100;
            case 100:        //strafe to line of blocks to get close enough for color sensor
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
                    lastTime = getRuntime();
                    state = 220;
                    moveHelper.runWithoutEncoders();
                }
                advanceToStateAfterTime(998,5);
                break;
            case 220: // give robot time to stop before catching the encoder value
                advanceToStateAfterTime(225,.25);
                moveHelper.omniDrive(0,0,0);
                break;
            case 225:
                firstBlockDistance = moveHelper.getEncoderValue(); // a negative value
                moveHelper.resetEncoders();
                returndist = firstBlockDistance - BRIDGE_TRAVEL_DISTANCE - SKYSTONE_DISTANCE;
                state = 230;
            case 230:        //lower block Arm
                blockArmServoHelper.Close();
                advanceToStateAfterTime(240,.5);
                moveHelper.runWithoutEncoders(); // Turn off encoders so the next move can be a manual turn
                break;
            case 240:        //Strafe to separate from the line
                moveHelper.driveBySensor(imuHelper.getAngleInRadians()+NINETY_IN_RADIANS,sensorRange.getDistance(DistanceUnit.INCH), 0);
                advanceToStateAfterTime(250,2);
                break;
            case 250:
                moveHelper.resetEncoders();
                moveHelper.runWithoutEncoders();
                moveHelper.omniDrive(0,0,0);
                state = 260;
                break;
            case 260:        //run to pass the bridge for the first time
                int dist = BRIDGE_TRAVEL_DISTANCE - firstBlockDistance; // subtracting a negative - ie add
                // At first, go fast.  Then go more slowly at the other side
                if (moveHelper.GetBLMotorPosition() < dist/2){
                    moveHelper.driveBySensor(imuHelper.getAngleInRadians()+NINETY_IN_RADIANS,sensorRange.getDistance(DistanceUnit.INCH),0.8);
                } else {
                    moveHelper.driveBySensor(imuHelper.getAngleInRadians()+NINETY_IN_RADIANS,sensorRange.getDistance(DistanceUnit.INCH),0.4);
                }
                if (moveHelper.GetBLMotorPosition() > dist){
                    lastTime = getRuntime();
                    state = 280;
                }
                advanceToStateAfterTime(280,2.5);
                break;
            case 280: //Block Arm up
                moveHelper.omniDrive(0,0,0);
                moveHelper.resetEncoders();
                blockArmServoHelper.Open();
                moveHelper.runWithoutEncoders();
                if (firstBlockDistance < -450){ //If third position
                    advanceToStateAfterTime(350,1); //Park
                }else {
                    advanceToStateAfterTime(300,1); //Get another skystone
                }
                break;
            case 300: // move back to other side
                moveHelper.driveBySensor(imuHelper.getAngleInRadians()+NINETY_IN_RADIANS,sensorRange.getDistance(DistanceUnit.INCH),-0.4);
                if (moveHelper.GetBLMotorPosition() < returndist){
                    lastTime = getRuntime();
                    state = 305;
                }
                advanceToStateAfterTime(305,3.5);       //Here!!!   2.5 #1
                break;
            case 305:
                moveHelper.omniDrive(0,0,0);
                moveHelper.resetEncoders();
                moveHelper.runWithoutEncoders();
                advanceToStateAfterTime(310,.5);       //Here!!!      .25  #1
                break;
            case 310: // creep to blocks to find range
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
                    lastTime = getRuntime();
                    moveHelper.runWithoutEncoders();
                    state = 320;
                }
                advanceToStateAfterTime(998,4);
                break;
            case 320:        //lower block Arm
                blockArmServoHelper.Close();
                moveHelper.omniDrive(0,0,0);
                advanceToStateAfterTime(325,.5);
                break;
            case 325:
                secondBlockDistance = moveHelper.getEncoderValue();
                returndist += secondBlockDistance;
                moveHelper.resetEncoders();
                moveHelper.runWithoutEncoders();
                state = 330;
                break;
            case 330:        //Strafe to separate from the line
                moveHelper.driveBySensor(imuHelper.getAngleInRadians()+NINETY_IN_RADIANS,sensorRange.getDistance(DistanceUnit.INCH), 0);
                advanceToStateAfterTime(335,1.5);
                break;
            case 335:
                moveHelper.omniDrive(0,0,0);
                advanceToStateAfterTime(337,.25);
                break;
            case 337:
                moveHelper.resetEncoders();
                moveHelper.runWithoutEncoders();
                state = 340;
                break;
            case 340:        //run to pass the bridge
                moveHelper.driveBySensor(imuHelper.getAngleInRadians()+NINETY_IN_RADIANS,sensorRange.getDistance(DistanceUnit.INCH),0.6);
                if (moveHelper.GetBLMotorPosition() > -returndist){
                    lastTime = getRuntime();
                    state = 345;
                }
                advanceToStateAfterTime(345,3);
                break;
            case 345: //Block Arm up
                moveHelper.omniDrive(0,0,0);
                moveHelper.resetEncoders();
                moveHelper.runWithoutEncoders();
                blockArmServoHelper.Open();
                advanceToStateAfterTime(350,.5);
                moveHelper.rangedTarget = 26;
                break;
            case 350: //Move to park on blue line.
                moveHelper.driveBySensor(imuHelper.getAngleInRadians()+NINETY_IN_RADIANS,sensorRange.getDistance(DistanceUnit.INCH),-0.2);
                if (sensorColor.blue() > 30 && sensorColor.green() > 0 && sensorColor.red() > 0) {
                    double blueToGreen = (double) sensorColor.blue() / sensorColor.green();
                    double blueToRed = (double) sensorColor.blue() / sensorColor.red();
                    telemetry.addData("Red Ratio ", blueToRed);
                    telemetry.addData("Green Ratio  ", blueToGreen);

                    if (blueToGreen > 1.2 && blueToRed > 1.5) {
                        state = 360;
                    }
                }
                advanceToStateAfterTime(360,5);
                break;
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
        telemetry.addData("returndist",returndist);
        telemetry.addData("FirstBlock",firstBlockDistance);
        telemetry.addData("SecondBlock", secondBlockDistance);
        telemetry.update();
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
        if (totalBlockSensorValues < 1300){
            return false;
        }
        return true;
    }
}