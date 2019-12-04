package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.awt.font.NumericShaper;

public class PickupArmHelper extends OperationHelper {
    private DcMotor elevationMotor;
    private DcMotor extensionMotor;
    protected Servo gripServo;
    protected Servo wristHorizontalServo;
    protected Servo wristVerticalServo;
    public static double GRIP_SERVO_MIN = 0;
    public static double GRIP_SERVO_MAX = 1;
    public static double WRIST_HORIZONTAL_SERVO_MAX = 1;
    public static double WRIST_HORIZONTAL_SERVO_MIN = 0;
    public static double WRIST_VERTICAL_SERVO_MAX = 1;
    public static double WRIST_VERTICAL_SERVO_MIN = 0;
    public static double GRIP_SERVO_SPEED = .05;
    public static double WRIST_HORIZONTAL_SERVO_SPEED = .05;
    public static double WRIST_VERTICAL_SERVO_SPEED = .05;
    public static double EXTENSION_SPEED = .5;
    public static double BLOCK_ARM_SERVO_CLOSED = 1;
    public static double BLOCK_ARM_SERVO_OPEN = 0;

    public PickupArmHelper(Telemetry t, HardwareMap h)
    {
        super(t, h);
    }

    public void init( ) {
        if (hardwareMap.servo.contains("gripservo")) {
            gripServo = hardwareMap.servo.get("gripservo");
            if (gripServo != null) {
                // gripServo.setPosition(BLOCK_ARM_SERVO_OPEN);
            }
        }
        if (hardwareMap.servo.contains("wristhorizontalservo")) {
            wristHorizontalServo = hardwareMap.servo.get("wristhorizontalservo");
            if (wristHorizontalServo != null) {
                // wristHorizontalServo.setPosition(BLOCK_ARM_SERVO_OPEN);
            }
        }
        if (hardwareMap.servo.contains("wristverticalservo")) {
            wristVerticalServo = hardwareMap.servo.get("wristverticalservo");
            if (wristVerticalServo != null) {
                // wristVerticalServo.setPosition(BLOCK_ARM_SERVO_OPEN);
            }
        }
        if (hardwareMap.dcMotor.contains("elevationmotor")) {
            elevationMotor = hardwareMap.dcMotor.get("elevationmotor");
        }
        if (hardwareMap.dcMotor.contains("extensionmotor")) {
            extensionMotor = hardwareMap.dcMotor.get("extensionmotor");
        }

    }

    public void checkTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad2.right_trigger != 0){
            double current = gripServo.getPosition();
            current += GRIP_SERVO_SPEED;
            current = Range.clip(current, GRIP_SERVO_MIN, GRIP_SERVO_MAX );
            gripServo.setPosition(current);
        }
        if (gamepad2.left_trigger != 0){
            double current = gripServo.getPosition();
            current += GRIP_SERVO_SPEED;
            current = Range.clip(current, GRIP_SERVO_MIN, GRIP_SERVO_MAX );
            gripServo.setPosition(current);
        }
        if (gamepad2.right_stick_x != 0){
            double current = wristHorizontalServo.getPosition();
            current += WRIST_HORIZONTAL_SERVO_SPEED * gamepad2.right_stick_x;
            current = Range.clip(current, WRIST_HORIZONTAL_SERVO_MIN, WRIST_HORIZONTAL_SERVO_MAX );
            wristHorizontalServo.setPosition(current);
        }
        if (gamepad2.right_stick_y != 0){
            double current = wristVerticalServo.getPosition();
            current += WRIST_VERTICAL_SERVO_SPEED * gamepad2.right_stick_y;
            current = Range.clip(current, WRIST_VERTICAL_SERVO_MIN, WRIST_VERTICAL_SERVO_MAX );
            wristVerticalServo.setPosition(current);
        }
        double LY = Range.clip(gamepad2.left_stick_y, -1, 1);
        elevationMotor.setPower(LY);

        if (gamepad2.dpad_up) {
            extensionMotor.setPower(EXTENSION_SPEED);
        }
        else if (gamepad2.dpad_down) {
            extensionMotor.setPower(-EXTENSION_SPEED);
        }
        else {
            extensionMotor.setPower(0);
        }
    }
}
