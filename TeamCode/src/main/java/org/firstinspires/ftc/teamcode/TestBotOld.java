package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@TeleOp(name="TestBotOld", group="TeleOp")
public class TestBotOld extends OpMode{

    MoveHelper moveHelper;
    DistanceSensor sensorRange;

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        sensorRange = hardwareMap.get(DistanceSensor.class, "range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        moveHelper.resetEncoders();
        moveHelper.runUsingEncoders();
    }

    @Override
    public void loop() {
        double shortRange = 4;
        double longRange = 6;
        double range = sensorRange.getDistance(DistanceUnit.INCH);
        moveHelper.checkTeleOp(gamepad1,gamepad2);

 /*       if (range > longRange){                       Laser Rangefinder Wallfollow Logic
            moveHelper.rturn(1);
        }
        else if (range < shortRange){
            moveHelper.lturn(1);
        }
        else {
            moveHelper.checkTeleOp(gamepad1,gamepad2);
        }

  */

        if (gamepad1.left_stick_button){        //Resets Encoder Values With Left Stick Button
            moveHelper.resetEncoders();
            moveHelper.runUsingEncoders();
        }
        if (gamepad1.right_stick_button){
            moveHelper.runWithoutEncoders();
        }

        //moveHelper.showEncoderValues();
        telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.update();

    }
}
