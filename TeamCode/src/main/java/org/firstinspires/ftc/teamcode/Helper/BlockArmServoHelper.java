package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BlockArmServoHelper extends OperationHelper {
    protected Servo blockArmServo;
    public static double BLOCK_ARM_SERVO_CLOSED = 0.75;
    public static double BLOCK_ARM_SERVO_OPEN = 0.12;

    public BlockArmServoHelper(Telemetry t, HardwareMap h)
    {
        super(t, h);
    }

    public void init( ) {
        if (hardwareMap.servo.contains("blockarm")) {
            blockArmServo = hardwareMap.servo.get("blockarm");
            if (blockArmServo != null) {
               // blockArmServo.setPosition(BLOCK_ARM_SERVO_OPEN);
            }
        }

    }

    public void GoToPosition(double position) {
        if (blockArmServo != null) {
            blockArmServo.setPosition(position);
        } else {
            telemetry.addData("Block Arm servo not active","");
        }
    }

    public void Close(){
        if (blockArmServo != null) {
            blockArmServo.setPosition(BLOCK_ARM_SERVO_CLOSED);
        } else {
            telemetry.addData("Block Arm servo not active","");
        }
    }
    public void Open(){
        if (blockArmServo != null) {
            blockArmServo.setPosition(BLOCK_ARM_SERVO_OPEN);
        } else {
            telemetry.addData("Block Arm servo not active","");
        }
    }
}
