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
    protected Servo gripServo;  // Continuous rotation
    protected Servo wristHorizontalServo;
    protected Servo wristVerticalServo;
    public static double WRIST_HORIZONTAL_SERVO_MAX = 1;
    public static double WRIST_HORIZONTAL_SERVO_MIN = 0;
    public static double WRIST_VERTICAL_SERVO_MAX = 1;
    public static double WRIST_VERTICAL_SERVO_MIN = .5;
    public static double GRIP_SERVO_MAX = 0.5;
    public static double GRIP_SERVO_MIN = 0.0;
    public static double GRIP_SERVO_SPEED = 1.0;
    public static double WRIST_HORIZONTAL_SERVO_SPEED = .05;
    public static double WRIST_VERTICAL_SERVO_SPEED = .05;
    public static double EXTENSION_SPEED = .5;
    public static double ELEVATION_SPEED = 20;
    private static final int LOWER_LIMIT = -800; // maximum "height" for elevation arm
    private int desiredPosition = 0;


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
            wristHorizontalServo.setPosition(WRIST_HORIZONTAL_SERVO_MIN);
           if (wristHorizontalServo != null) {
                wristHorizontalServo.setPosition(WRIST_HORIZONTAL_SERVO_MIN);
            }
        }
        if (hardwareMap.servo.contains("wristverticalservo")) {
            wristVerticalServo = hardwareMap.servo.get("wristverticalservo");
            wristVerticalServo.setPosition(WRIST_VERTICAL_SERVO_MIN);
            if (wristVerticalServo != null) {
                wristVerticalServo.setPosition(WRIST_VERTICAL_SERVO_MIN);
            }
        }
        if (hardwareMap.dcMotor.contains("elevationmotor")) {
            elevationMotor = hardwareMap.dcMotor.get("elevationmotor");
        }
        if (hardwareMap.dcMotor.contains("extensionmotor")) {
            extensionMotor = hardwareMap.dcMotor.get("extensionmotor");
        }
        elevationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        //Turn into if statement
        elevationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void checkMissingComponents() {
        if (gripServo == null) {
            telemetry.addData("gripServo not initialized","");
        }
        if (wristHorizontalServo == null) {
            telemetry.addData("wristHorizontalServo not initialized","");
        }
        if (wristVerticalServo == null) {
            telemetry.addData("wristVerticalServo not initialized","");
        }
        if (elevationMotor == null) {
            telemetry.addData("elevationMotor not initialized","");
        }
        if (extensionMotor == null) {
            telemetry.addData("extensionMotor not initialized","");
        }
        telemetry.update();
    }

    public void checkTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad2.right_trigger != 0){
            double current = gripServo.getPosition();
            current += GRIP_SERVO_SPEED * gamepad2.right_trigger;
            current = Range.clip(current, GRIP_SERVO_MIN, GRIP_SERVO_MAX );
            gripServo.setPosition(current);

            telemetry.addData("Right trigger", gamepad2.right_trigger);
            telemetry.addData("Grip position", gripServo.getPosition());
        }
        if (gamepad2.left_trigger != 0){
            double current = gripServo.getPosition();
            current -= GRIP_SERVO_SPEED * gamepad2.left_trigger;
            current = Range.clip(current, GRIP_SERVO_MIN, GRIP_SERVO_MAX );
            gripServo.setPosition(current);
            telemetry.addData("Left trigger", gamepad2.left_trigger);
            telemetry.addData("Grip position", gripServo.getPosition());
        }
        if (gamepad2.right_stick_x != 0){
            double current = wristHorizontalServo.getPosition();
            current += WRIST_HORIZONTAL_SERVO_SPEED * gamepad2.right_stick_x;
            current = Range.clip(current, WRIST_HORIZONTAL_SERVO_MIN, WRIST_HORIZONTAL_SERVO_MAX );
            wristHorizontalServo.setPosition(current);
            telemetry.addData("Right Stick x", gamepad2.right_stick_x);
            telemetry.addData("Wrist horizontal position", wristHorizontalServo.getPosition());

        }
        if (gamepad2.right_stick_y != 0){
            double current = wristVerticalServo.getPosition();
            current += WRIST_VERTICAL_SERVO_SPEED * gamepad2.right_stick_y;
            current = Range.clip(current, WRIST_VERTICAL_SERVO_MIN, WRIST_VERTICAL_SERVO_MAX );
            wristVerticalServo.setPosition(current);
            telemetry.addData("Right stick y", gamepad2.right_stick_y);
            telemetry.addData("Wrist vertical position", wristVerticalServo.getPosition());
        }
        //double LY = Range.clip(gamepad2.left_stick_y, -1, 1);
        //elevationMotor.setPower(LY * ELEVATION_SPEED);
        if (gamepad2.left_stick_y != 0){
            raise(gamepad2.left_stick_y);
        }

        if (gamepad2.dpad_up) {
            extensionMotor.setPower(EXTENSION_SPEED);
        }
        else if (gamepad2.dpad_down) {
            extensionMotor.setPower(-EXTENSION_SPEED);
        }
        else {
            extensionMotor.setPower(0);
        }
        checkMissingComponents();
    }

    public int getPosition() {
            return elevationMotor.getCurrentPosition();
    }

    public void raise(double raiseAmount){
        desiredPosition += raiseAmount * ELEVATION_SPEED;
        if (desiredPosition < LOWER_LIMIT) {
            desiredPosition = LOWER_LIMIT;
        }
        elevationMotor.setTargetPosition((int)desiredPosition);
        elevationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevationMotor.setPower(.5);
    }

    public void holdPosition() {
        elevationMotor.setTargetPosition(desiredPosition);
        elevationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevationMotor.setPower(.6);
    }

    public void resetHold () {
        elevationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        desiredPosition = elevationMotor.getCurrentPosition();
        elevationMotor.setPower(0);
    }

    public void setHold() {
        desiredPosition = elevationMotor.getCurrentPosition();
    }
}
