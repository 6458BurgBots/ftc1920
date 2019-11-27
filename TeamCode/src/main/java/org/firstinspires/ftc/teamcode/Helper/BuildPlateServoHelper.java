package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BuildPlateServoHelper extends OperationHelper {
    protected Servo plateArmServo;
    public static double SAMPLE_SERVO_CLOSED = 1;
    public static double SAMPLE_SERVO_OPEN =  .5;

    public BuildPlateServoHelper(Telemetry t, HardwareMap h)
    {
        super(t, h);
    }

    public void init( ) {
        if (hardwareMap.servo.contains("platearm")) {
            plateArmServo = hardwareMap.servo.get("platearm");
            if (plateArmServo != null) {
                plateArmServo.setPosition(SAMPLE_SERVO_OPEN);
            }
        }

    }

    public void Close(){
        if (plateArmServo != null) {
            plateArmServo.setPosition(SAMPLE_SERVO_CLOSED);
        }
    }
    public void Open(){
        if (plateArmServo != null) {
            plateArmServo.setPosition(SAMPLE_SERVO_OPEN);
        }
    }


    }
