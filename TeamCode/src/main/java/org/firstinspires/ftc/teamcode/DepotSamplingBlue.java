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

    public static double SAMPLE_SERVO_CLOSED = 1;
    public static double SAMPLE_SERVO_OPEN = .5;
    MoveHelper moveHelper;
    DistanceSensor sensorRange;
    ColorSensor sensorColor;
    ColorSensor blockColor;
    BlockArmServoHelper blockArmServoHelper;
    IMUHelper imuHelper;
    double lastTime;
    //protected Servo plateArmServo;
    public int long_move = -6250;
    int state = 0;

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        //sensorRange = hardwareMap.get(DistanceSensor.class, "range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;
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

                // Adjust angle so that we can move straight to the wall
            case 60:
                imuHelper.turnTo(-90);
                advanceToStateAfterTime(100, 1);
                break;

            case 100:        //move back to the wall
                moveHelper.encoderPowerLevel = .2;
                moveHelper.runMotorsToPosition(600,-600,600,-600);
                if (isCloseToBlock(blockColor)){
                    state = 200;
                }
                advanceToStateAfterTime(997,3); //failsafe
                break;
            case 200:
                moveHelper.runWithoutEncoders();
                state = 205;
                break;

            // Adjust angle so that we can move straight to the wall
            case 205:
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
                    state = 220;
                }
                advanceToStateAfterTime(998,4);
                break;
            case 220:
                moveHelper.resetEncoders();
                state = 230;
                break;

            case 230:        //lower block Arm
                blockArmServoHelper.Close();
                advanceToStateAfterTime(240,1);
                break;

            case 240:        //Strafe to separate from the line
                moveHelper.encoderPowerLevel = .7;
                moveHelper.runMotorsToPosition(-1000,1000,-1000,1000);
                advanceToStateAfterTime(250,3);
                break;
            case 250:
                moveHelper.runWithoutEncoders(); // Turn off encoders so the next move can be a manual turn
                moveHelper.encoderPowerLevel = 1;
                state = 255;
                break;
            case 255:
                imuHelper.turnTo(-90);
                advanceToStateAfterTime(257, 1.5);
                break;

            case 257:
                moveHelper.resetEncoders();
                moveHelper.runUsingEncoders();
                state = 260;
                break;

            case 260:        //run to pass the bridge THIS IS A COPY/PASTE FROM BlueDepotSide
                moveHelper.runMotorsToPosition(2500, 2500, 2500, 2500);
                advanceToStateAfterTime(270,3);
                break;
            case 270:
                moveHelper.resetEncoders();
                state = 280;
                break;

            case 280: //Block Arm up
                blockArmServoHelper.Open();
                advanceToStateAfterTime(290,2);
                break;
            case 290:
                telemetry.addData("End Program", "");
                break;
            case 997:
                telemetry.addData("End: didn't sense stone line", "");
                break;
            case 998:
                telemetry.addData("End: never found skystone", "");
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