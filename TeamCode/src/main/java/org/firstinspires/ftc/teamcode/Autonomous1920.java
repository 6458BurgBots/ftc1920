package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@Autonomous(name="Autonomous1920", group="Autonomous")
public class Autonomous1920 extends OpMode{

    MoveHelper moveHelper;
    DistanceSensor sensorRange;
    int state = 0;
    double lastTime;

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        sensorRange = hardwareMap.get(DistanceSensor.class, "range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        moveHelper.resetEncoders();
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
    @Override
    public void loop() {
        telemetry.addData("state",state);
        telemetry.update();
        switch (state) {
            case 0:
                lastTime = getRuntime();
                state = 10;
                break;
            case 10:
                moveHelper.driveForward(1);
                if (getRuntime()-lastTime > 2){
                    state = 20;
                    lastTime = getRuntime();
                }
                break;
            case 20:
                moveHelper.driveForward(0);
                if (getRuntime()-lastTime > .25) {
                    state = 30;
                }
                break;
            case 30:
                moveHelper.driveForward(-1);
                advanceToStateAfterTime(40,2);
                break;
            case 40:
                moveHelper.driveForward(0);
                if (getRuntime()-lastTime > .25) {
                    state = 50;
                }
                break;
        }
    }
}
