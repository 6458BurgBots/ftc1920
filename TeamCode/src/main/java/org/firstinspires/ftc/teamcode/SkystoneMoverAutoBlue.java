////////////////CRACKED MAGNET, ROBOT STRAFING MESSED UP

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Helper.BlockArmServoHelper;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@Autonomous(name= "SkystoneMoverAutoBlue", group="Autonomous")
public class SkystoneMoverAutoBlue extends OpMode {

    MoveHelper moveHelper;
    int state = 0;
    double lastTime;
    ColorSensor sensorColor;
    BlockArmServoHelper blockArmServoHelper;

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        blockArmServoHelper = new BlockArmServoHelper(telemetry, hardwareMap);
        blockArmServoHelper.init();
        moveHelper.resetEncoders();
        moveHelper.runUsingEncoders();
        //sensorColor = hardwareMap.get(ColorSensor.class, "colorsensor");
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
            case 10: //Move towards blocks, strafe left.
                moveHelper.runMotorsToPosition(1700,-1700,1700,-1700);
                advanceToStateAfterTime(15,3);
                break;
            case 15: //Stop.
                moveHelper.resetEncoders();
                state = 20;
                break;
            case 20: //Move arm down
                blockArmServoHelper.Close();
                advanceToStateAfterTime(30,.25);
                break;
            case 30: //Move towards wall, strafe right.
                moveHelper.runMotorsToPosition(-500,500,-500,500);
                advanceToStateAfterTime(35,2);
                break;
            case 35: //Stop.
                moveHelper.resetEncoders();
                state = 40;
                break;
            case 40: //Move under and across middle, blue tape, move backwards.
                moveHelper.runMotorsToPosition(5000,5000,5000,5000);
                advanceToStateAfterTime(45,5);
                break;
            case 45: //Stop.
                moveHelper.resetEncoders();
                state = 50;
                break;
            case 50: //Lift arm
                blockArmServoHelper.Open();
                advanceToStateAfterTime(60,.25);
                break;
            case 60: //Move forwards, back across mid line.
                moveHelper.runMotorsToPosition(-5000,-5000,-5000,-5000);
                advanceToStateAfterTime(65,5);
                break;
            case 65: //Stop.
                moveHelper.resetEncoders();
                break;
        }
    }
}
