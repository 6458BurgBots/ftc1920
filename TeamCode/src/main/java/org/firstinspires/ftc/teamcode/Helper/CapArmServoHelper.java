package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CapArmServoHelper extends OperationHelper {
    protected Servo capArmServo;
    public static double CAP_ARM_SERVO_CLOSED = 0.78;
    public static double CAP_ARM_SERVO_OPEN = 0.12;

    public CapArmServoHelper(Telemetry t, HardwareMap h)
    {
        super(t, h);
    }

    public void init( ) {
        if (hardwareMap.servo.contains("caparm")) {
            capArmServo = hardwareMap.servo.get("caparm");
            if (capArmServo != null) {
               // capArmServo.setPosition(CAP_ARM_SERVO_OPEN);
            }
        }

    }

    public void GoToPosition(double position) {
        if (capArmServo != null) {
            capArmServo.setPosition(position);
        } else {
            telemetry.addData("Cap Arm servo not active","");
        }
    }

    public void Close(){
        if (capArmServo != null) {
            capArmServo.setPosition(CAP_ARM_SERVO_CLOSED);
        } else {
            telemetry.addData("Cap Arm servo not active","");
        }
    }
    public void Open(){
        if (capArmServo != null) {
            capArmServo.setPosition(CAP_ARM_SERVO_OPEN);
        } else {
            telemetry.addData("Cap Arm servo not active","");
        }
    }
}
