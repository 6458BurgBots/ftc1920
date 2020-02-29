package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This is the version that works with the squeeze arm
public class StateArmHelper extends OperationHelper {
    private DcMotor elevationMotor;
    private DcMotor tapeMotor;
    protected Servo gripServo;
    protected Servo wristHorizontalServo;

    public static double WRIST_HORIZONTAL_SERVO_MAX = 1;
    public static double WRIST_HORIZONTAL_SERVO_MIN = 0;                  //Here
    public static double WRIST_HORIZONTAL_SERVO_SPEED = .4;
    public static double WRIST_HORIZONTAL_SERVO_LEVEL = 1;
    public static double GRIP_SERVO_MAX = .5;                             //Here
    public static double GRIP_SERVO_MIN = 0.0;
    public static double GRIP_SERVO_SPEED = 1.0;
    public static int TAPE_OPEN_POSITION = 200;
    public static int TAPE_CLOSED_POSITION = 0;
//    public static boolean TAPE_MOTOR = false;
    public static double ELEVATION_SPEED = 20;
    private static final int LOWER_LIMIT = -800; // maximum "height" for elevation arm               //Here
    private int desiredPosition = 0;


    public StateArmHelper(Telemetry t, HardwareMap h)
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
                wristHorizontalServo.setPosition(WRIST_HORIZONTAL_SERVO_MAX);
            }
        }
        if (hardwareMap.dcMotor.contains("elevationmotor")) {
            elevationMotor = hardwareMap.dcMotor.get("elevationmotor");
        }

        if (hardwareMap.dcMotor.contains("tapemotor")) {
            tapeMotor = hardwareMap.dcMotor.get("tapemotor");
        }
        elevationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tapeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void checkMissingComponents() {
        if (gripServo == null) {
            telemetry.addData("gripServo not initialized","");
        }
        if (wristHorizontalServo == null) {
            telemetry.addData("wristVerticalServo not initialized","");
        }
        if (elevationMotor == null) {
            telemetry.addData("elevationMotor not initialized","");

        }
        if (tapeMotor == null) {
            telemetry.addData("tapeMotor not initialized","");

        }

        telemetry.update();
    }

    public void checkTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad2.right_trigger != 0) {
            double current = gripServo.getPosition();
            current += GRIP_SERVO_SPEED * gamepad2.right_trigger;
            current = Range.clip(current, GRIP_SERVO_MIN, GRIP_SERVO_MAX);
            gripServo.setPosition(current);

        }
        if (gamepad2.left_trigger != 0) {
            double current = gripServo.getPosition();
            current -= GRIP_SERVO_SPEED * gamepad2.left_trigger;
            current = Range.clip(current, GRIP_SERVO_MIN, GRIP_SERVO_MAX);
            gripServo.setPosition(current);
            telemetry.addData("Left trigger", gamepad2.left_trigger);
            telemetry.addData("Grip position", gripServo.getPosition());
        }
        if (gamepad2.right_stick_x != 0) {
            double current = wristHorizontalServo.getPosition();
            current -= WRIST_HORIZONTAL_SERVO_SPEED * gamepad2.right_stick_x;
            current = Range.clip(current, WRIST_HORIZONTAL_SERVO_MIN, WRIST_HORIZONTAL_SERVO_MAX);
            wristHorizontalServo.setPosition(current);
            telemetry.addData("Right stick x", gamepad2.right_stick_x);
            telemetry.addData("Current", current);
            telemetry.update();
        }

        if (gamepad1.right_bumper) {
            tapeMotor.setPower(1);
        }

        else if (gamepad1.left_bumper){
            tapeMotor.setPower(-0.5);                                  //HERE
        }

        else {
            tapeMotor.setPower(0);
        }

//        tapeMotor.setPower(-gamepad2.right_stick_y);
        elevationMotor.setPower(gamepad2.left_stick_y);

            //double LY = Range.clip(gamepad2.left_stick_y, -1, 1);
            //elevationMotor.setPower(LY * ELEVATION_SPEED);

//        if (gamepad2.left_stick_y != 0){
//            raise(gamepad2.left_stick_y);
//        }
//
//        if (gamepad2.y) {
//            levelGrip();
//        }
            checkMissingComponents();


    }




        public int getPosition() {
            return elevationMotor.getCurrentPosition();
    }

//    public void raise(double raiseAmount){
//        desiredPosition += raiseAmount * ELEVATION_SPEED;
//        if (desiredPosition < LOWER_LIMIT) {
//            desiredPosition = LOWER_LIMIT;
//        }
//        elevationMotor.setTargetPosition((int)desiredPosition);
//        elevationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elevationMotor.setPower(.5);
//    }





    public void levelGrip( ){
        wristHorizontalServo.setPosition(WRIST_HORIZONTAL_SERVO_LEVEL);
    }

}
