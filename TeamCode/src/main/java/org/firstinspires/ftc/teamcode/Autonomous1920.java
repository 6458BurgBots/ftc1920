package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@Autonomous(name="Autonomous1920", group="Autonomous")
public class Autonomous1920 extends OpMode{

    MoveHelper moveHelper;
    DistanceSensor sensorRange;
    int state = 0;
    double lastTime;
    ColorSensor sensorColor;

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        sensorRange = hardwareMap.get(DistanceSensor.class, "range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        moveHelper.resetEncoders();
        moveHelper.runUsingEncoders();
        sensorColor = hardwareMap.get(ColorSensor.class, "colorsensor");
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
                state = 10;
                break;
            case 10: //Move forward to blocks.
                //moveHelper.driveForward(-1);
                moveHelper.runMotorsToPosition(-1770,-1770,-1770,-1770);
                advanceToStateAfterTime(15, 2);
                break;
            case 15:
                moveHelper.resetEncoders();
                state = 20;
                break;
            case 20: //move backward towards wall.
                moveHelper.runMotorsToPosition(1500,1500,1500,1500);
                advanceToStateAfterTime(25, 2);
                break;
            case 25:
                moveHelper.resetEncoders();
                state = 30;
                break;
            case 30: //Turn 90 degrees to the left
                moveHelper.runMotorsToPosition(1000,-1000,-1000,1000);
                advanceToStateAfterTime(35,2);
                break;
            case 35:
                moveHelper.resetEncoders();
                state = 40;
                break;
            case 40: //move across mid line and towards building site.
                moveHelper.runMotorsToPosition(-6500,-6500,-6500,-6500);
                advanceToStateAfterTime(45,5);
                break;
            case 45:
                moveHelper.resetEncoders();
                state = 50;
                break;
            case 50: //Turn 90 degrees to the left towards the building plate.
                moveHelper.runMotorsToPosition(-1000,1000,1000,-1000);
                advanceToStateAfterTime(55,2);
                break;
            case 55:
                moveHelper.resetEncoders();
                state = 60;
                break;
            case 60: //Move forward towards building plate.
                moveHelper.runMotorsToPosition(-1500,-1500,-1500,-1500);
                advanceToStateAfterTime(65,2);
                break;
            case 65:
                moveHelper.resetEncoders();
                state = 70;
                break;
            case 70: //Move backward with building plate.
                moveHelper.runMotorsToPosition(1500,1500,1500,1500);
                advanceToStateAfterTime(75, 2);
                break;
            case 75:
                //moveHelper.resetEncoders();
                moveHelper.runWithoutEncoders();
                state = 80;
                break;
            case 80://need to change encoder values. Strafe towards center of field.
                moveHelper.omniDrive(.3,0,0);
                if (sensorColor.blue() > 30 && sensorColor.green() > 0 && sensorColor.red() > 0)
                {
                    double blueToGreen = (double)sensorColor.blue() / sensorColor.green();
                    double blueToRed = (double)sensorColor.blue() / sensorColor.red();
                    telemetry.addData("Red Ratio ", blueToRed);
                    telemetry.addData("Green Ratio  ", blueToGreen);

                    if (blueToGreen > 1.3 && blueToRed > 2) {
                        state = 90;
                    }
                }
                break;

            case 90:
                moveHelper.omniDrive(0,0,0);
                state = 100;
                break;
            case 100:
                /*need to add color sensor stuff for moving to the middle of the field.
                Move the color sensor closer to the floor so we can have more accurate readings
                We might have to turn instead of strife the whole was so that the color sensor stops the robot over the line
                For Wesley: At beginning of practice on thursday take encoder values for strafe out of the corner. Also get encoder
                values for the move forward and the turn.

*/

    }
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("State", state);
        telemetry.update();
    }
}
