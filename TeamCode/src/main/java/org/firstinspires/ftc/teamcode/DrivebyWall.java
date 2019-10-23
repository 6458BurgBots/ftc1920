package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@TeleOp(name="DrivebyWall", group="TeleOp")
public class DrivebyWall extends OpMode{

    MoveHelper moveHelper;
    DistanceSensor sensorRange;

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        sensorRange = hardwareMap.get(DistanceSensor.class, "range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
    }

    @Override
    public void loop() {
        double shortRange = 4;
        double longRange = 6;
        double range = sensorRange.getDistance(DistanceUnit.INCH);

        if (range > longRange){
            moveHelper.rturn(1);
        }
        else if (range < shortRange){
            moveHelper.lturn(1);
        }
        else {
            moveHelper.checkTeleOp(gamepad1,gamepad2);

        }


        telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.update();

    }
}
