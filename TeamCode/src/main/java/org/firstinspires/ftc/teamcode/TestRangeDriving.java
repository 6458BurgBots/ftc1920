package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@TeleOp(name="TestRangeDriving", group="TeleOp")
public class TestRangeDriving extends OpMode {
    MoveHelper moveHelper;
    private DistanceSensor sensorRange;
    int count = 0;
    double target = 24;
    double scale = 24;

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        moveHelper.resetEncoders();
        moveHelper.runWithoutEncoders();
        moveHelper.displayMoveOutputs = false;
        sensorRange = hardwareMap.get(DistanceSensor.class, "range");
    }

    @Override
    public void loop() {
        double seconds = getRuntime();
        count = count + 1;
        double samplingRate = count / seconds;
        double heading = ((sensorRange.getDistance(DistanceUnit.INCH) - target)/scale);
        moveHelper.omniDrive(0, -.5, heading);



        telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.addData("samplingRate", samplingRate);
        telemetry.addData("heading", heading);
        telemetry.update();
    }


}
