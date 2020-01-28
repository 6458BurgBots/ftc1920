package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Helper.BlockArmServoHelper;
import org.firstinspires.ftc.teamcode.Helper.BuildPlateServoHelper;
import org.firstinspires.ftc.teamcode.Helper.ClawHelper;
import org.firstinspires.ftc.teamcode.Helper.IMUHelper;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@TeleOp(name="TestGrip", group="TeleOp")
public class TestGripPosition extends OpMode{

    public static double GRIP_SERVO_MAX = 0.5;
    public static double GRIP_SERVO_MIN = 0.0;
    public static double GRIP_SERVO_SPEED = 0.001;

    protected Servo gripServo;

    @Override
    public void init() {
        if (hardwareMap.servo.contains("gripservo")) {
            gripServo = hardwareMap.servo.get("gripservo");
            if (gripServo != null) {
                // gripServo.setPosition(BLOCK_ARM_SERVO_OPEN);
            }
        }
    }

    @Override
    public void loop() {
        double current = gripServo.getPosition();
        boolean changeMade = false;
        if (gamepad1.x) {
            current += GRIP_SERVO_SPEED;
            telemetry.addData("Button: ", "X");
            changeMade = true;
        }
        if (gamepad1.b) {
            current -= GRIP_SERVO_SPEED;
            telemetry.addData("Button: ", "B");
            changeMade = true;
        }
        if (changeMade) {
            //current = Range.clip(current, GRIP_SERVO_MIN, GRIP_SERVO_MAX);
            gripServo.setPosition(current);
        }
        telemetry.addData("Grip position", gripServo.getPosition());
        telemetry.update();
    }
}
