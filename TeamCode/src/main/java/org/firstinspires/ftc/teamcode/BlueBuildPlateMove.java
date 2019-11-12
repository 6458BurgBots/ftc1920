package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@Autonomous(name="BlueBuildPlateMove", group="Autonomous")
public class BlueBuildPlateMove extends OpMode{

    public static double SAMPLE_SERVO_CLOSED = 1;
    public static double SAMPLE_SERVO_OPEN = .5;
    MoveHelper moveHelper;
    DistanceSensor sensorRange;
    int state = 0;
    double lastTime;
    ColorSensor sensorColor;
    protected Servo plateArmServo;
    public int long_move = -4700;

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

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        //sensorRange = hardwareMap.get(DistanceSensor.class, "range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        moveHelper.resetEncoders();
        moveHelper.runUsingEncoders();
        sensorColor = hardwareMap.get(ColorSensor.class, "colorsensor");
        plateArmServo = hardwareMap.servo.get("platearm");
        plateArmServo.setPosition(SAMPLE_SERVO_OPEN);
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
                state = 60;
                break;
            // case 10: //Move forward to blocks.
            //moveHelper.driveForward(-1);
            // moveHelper.runMotorsToPosition(-1770,-1770,-1770,-1770);
            // advanceToStateAfterTime(15, 2);
            // break;
            // case 15:
            // moveHelper.resetEncoders();
            // state = 20;
            // break;
            // case 20: //move backward towards wall.
            // moveHelper.runMotorsToPosition(1500,1500,1500,1500);
            // advanceToStateAfterTime(25, 2);
            // break;
            // case 25:
            // moveHelper.resetEncoders();
            // state = 30;
            // break;
            case 20: //move forward away from wall.
                moveHelper.runMotorsToPosition(-300,-300,-300,-300);
                advanceToStateAfterTime(25, 2);
                break;
            case 25:
                moveHelper.resetEncoders();
                state = 30;
                break;
            case 30: //Turn 90 degrees to the left
                moveHelper.runMotorsToPosition(1020,-1020,-1020,1020);
                advanceToStateAfterTime(35,2);
                break;
            case 35:
                moveHelper.resetEncoders();
                state = 40;
                break;
            case 40: //move across mid line and towards building site.
                moveHelper.runMotorsToPosition(long_move,long_move,long_move,long_move);
                advanceToStateAfterTime(45,5);
                break;
            case 45:
                moveHelper.resetEncoders();
                state = 50;
                break;
            case 50: //Turn 90 degrees to the right towards the building plate.
                moveHelper.runMotorsToPosition(-1050,1050,1050,-1050);
                advanceToStateAfterTime(55,2);
                break;
            case 55:
                moveHelper.resetEncoders();
                state = 60;
                break;
            case 60: //Move forward towards building plate.
                moveHelper.encoderPowerLevel = .2;
                moveHelper.runMotorsToPosition(-2000,-2000,-2000,-2000);
                advanceToStateAfterTime(65,5);
                break;
            case 65:
                moveHelper.resetEncoders();
                moveHelper.encoderPowerLevel = 1;
                state = 66;
                break;
            case 66:
                plateArmServo.setPosition(SAMPLE_SERVO_CLOSED);
                advanceToStateAfterTime(70,1);
                break;
            case 70: //Move backward with building plate.
                moveHelper.encoderPowerLevel = .5;
                moveHelper.runMotorsToPosition(2850,2850,2850,2850);
                advanceToStateAfterTime(75, 2);
                break;
            case 75:
                moveHelper.encoderPowerLevel = 1;
                moveHelper.resetEncoders();
                state = 76;
                break;
            case 76:
                plateArmServo.setPosition(SAMPLE_SERVO_OPEN);
                advanceToStateAfterTime(80,1);
                break;
            case 80://Strafe towards center of field.
                moveHelper.runMotorsToPosition(-2630,2630,-2630,2630);
                advanceToStateAfterTime(85, 2);
                break;
            case 85:
                moveHelper.resetEncoders();
                state = 90;
                break;
            case 90://Move forward away from wall
                moveHelper.runMotorsToPosition(-300,-300,-300,-300);
                advanceToStateAfterTime(95, 1);
                break;
            case 95:
                moveHelper.resetEncoders();
                state = 100;
                break;
            case 100://Turn right towards middle of field
                moveHelper.runMotorsToPosition(-1050,1050,1050,-1050);
                advanceToStateAfterTime(115, 1);
                break;
            case 115:
              //  moveHelper.resetEncoders();
                moveHelper.runUsingEncoders();
                state = 120;
                break;
            case 120:
                moveHelper.omniDrive(0,-.25,0);
                if (sensorColor.blue() > 30 && sensorColor.green() > 0 && sensorColor.red() > 0)
                {
                    double blueToGreen = (double)sensorColor.blue() / sensorColor.green();
                    double blueToRed = (double)sensorColor.blue() / sensorColor.red();
                    telemetry.addData("Red Ratio ", blueToRed);
                    telemetry.addData("Green Ratio  ", blueToGreen);

                    if (blueToGreen > 1.3 && blueToRed > 2)
                    {
                        state = 130;
                    }
                }
                break;
            case 130:
                moveHelper.omniDrive(0,0,0);
                break;
                /*need to add color sensor stuff for moving to the middle of the field.
                Move the color sensor closer to the floor so we can have more accurate readings
                We might have to turn instead of strafe the whole way so that the color sensor stops the robot over the line

*/

    }
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("State", state);
        telemetry.update();
    }
}
