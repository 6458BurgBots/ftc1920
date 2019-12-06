package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BuildPlateServoHelper extends OperationHelper {
    protected Servo plateArmServo1;
    protected Servo plateArmServo2;
    public static double SAMPLE_SERVO_CLOSED = 1;
    public static double SAMPLE_SERVO_OPEN = .5;

    public BuildPlateServoHelper(Telemetry t, HardwareMap h) {
        super(t, h);
    }

    public void init() {
        if (hardwareMap.servo.contains("buildplateservo1")) {
            plateArmServo1 = hardwareMap.servo.get("buildplateservo1");
            if (plateArmServo1 != null) {
                plateArmServo1.setPosition(SAMPLE_SERVO_OPEN);
            }
        }
        if (hardwareMap.servo.contains("buildplateservo2")) {
            plateArmServo2 = hardwareMap.servo.get("buildplateservo2");
            if (plateArmServo2 != null) {
                plateArmServo2.setPosition(SAMPLE_SERVO_OPEN);
            }
        }
        Open();
    }

    public void Close() {
        if (plateArmServo1 != null) {
            plateArmServo1.setPosition(SAMPLE_SERVO_CLOSED);
        } else {
            telemetry.addData("plateArmServo1 not initialized", "");

        }
        if (plateArmServo2 != null) {
            plateArmServo2.setPosition(SAMPLE_SERVO_CLOSED);
        } else {
            telemetry.addData("plateArmServo2 not initialized", "");
        }
    }

    public void Open() {
        if (plateArmServo1 != null) {
            plateArmServo1.setPosition(SAMPLE_SERVO_OPEN);
        } else {
            telemetry.addData("plateArmServo1 not initialized", "");

        }
        if (plateArmServo2 != null) {
            plateArmServo2.setPosition(SAMPLE_SERVO_OPEN);
        } else {
            telemetry.addData("plateArmServo2 not initialized", "");
        }
    }

    public void checkTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.dpad_up) {
            Open();
        }
        if (gamepad1.dpad_down) {
            Close();
        }
    }
}
