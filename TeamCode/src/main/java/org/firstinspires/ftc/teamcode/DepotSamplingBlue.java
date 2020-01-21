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
                advanceToStateAfterTime(25, 1.5);
                break;
            case 25:
                moveHelper.resetEncoders();
                state = 30;
                break;

            case 30:        //left turn to line up for color sensing
                moveHelper.runMotorsToPosition(750,-750,-750,750);
                advanceToStateAfterTime(35,1);
                break;

            case 35:
                moveHelper.resetEncoders();
                state = 40;
                break;

            case 40:        //move back to the wall
                moveHelper.encoderPowerLevel = .3;
                moveHelper.runMotorsToPosition(600,-600,600,-600);
                if (isCloseToBlock(blockColor)){
                    state = 45;
                }
                advanceToStateAfterTime(997,3); //failsafe
                break;
            case 45:
                moveHelper.resetEncoders();
                state = 50;
                break;

            case 50:        //light logic and move to sense
                moveHelper.runMotorsToPosition(-1200,-1200,-1200,-1200);
                if(isSkyStone(blockColor)){
                    state = 65;
                }
                else
                advanceToStateAfterTime(998,4);
                break;

         /*   case 55:        // run motors the length of one block
                moveHelper.runMotorsToPosition(-360,-360,-360,-360);
                state = 50;
                break;
            case 60:        //move to line up
                moveHelper.runMotorsToPosition(0,0,0,0);
                advanceToStateAfterTime(65,2);
                break; */

            case 65:
                moveHelper.resetEncoders();
                state = 70;
                break;

            case 70:        //lower block Arm
                blockArmServoHelper.Close();
                advanceToStateAfterTime(80,1);
                break;

            case 80:        //Strafe to separate from the line
                moveHelper.runMotorsToPosition(-800,800,-800,800);
                advanceToStateAfterTime(85,1.5);
                break;

            case 85:
                moveHelper.resetEncoders();
                moveHelper.encoderPowerLevel = 1;
                state = 90;
                break;

            case 90:        //run to pass the bridge THIS IS A COPY/PASTE FROM BlueDepotSide
                moveHelper.runMotorsToPosition(3000, 3000, 3000, 3000);
                advanceToStateAfterTime(95,3);
                break;

            case 95:
                moveHelper.resetEncoders();
                state = 105;
                break;
            case 105: //Block Arm up
                blockArmServoHelper.Open();
                advanceToStateAfterTime(110,2);
                break;
            case 110:
                break;
            case 115:
                break;
            case 997:
                break;
            case 998:
                break;
            case 999:
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