package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.IMUHelper;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@TeleOp(name="TestRangeDriving", group="TeleOp")
public class TestRangeDriving extends OpMode {
    MoveHelper moveHelper;
    IMUHelper imuHelper;
    private DistanceSensor sensorRange;
    int count = 0;
    double target = 24;
    double scale = 6; //98
    double angleScale = -1;

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        moveHelper.resetEncoders();
        moveHelper.runWithoutEncoders();
        moveHelper.displayMoveOutputs = false;
        sensorRange = hardwareMap.get(DistanceSensor.class, "range");
        imuHelper = new IMUHelper(telemetry, hardwareMap);
        imuHelper.init();
    }

    public void init_loop() {
        imuHelper.init_loop();
    }

    @Override
    public void loop() {
        double seconds = getRuntime();
        count = count + 1;
        double samplingRate = count / seconds;
        double heading = ((getTrueDistance() - target)/scale);
        double currentAngle = imuHelper.getAngleInRadians();
        if (currentAngle > .087 || currentAngle < -.087) {
            heading = 0;
        }
        moveHelper.omniDrive( heading,-.4,currentAngle*angleScale);



        telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.addData("trueDistance", String.format("%.01f in", getTrueDistance()));
        telemetry.addData("currentAngle", currentAngle);
        telemetry.addData("samplingRate", samplingRate);
        telemetry.update();
    }

    public double getTrueDistance() {
        double distance = sensorRange.getDistance(DistanceUnit.INCH);
        double gyroAngle = imuHelper.getAngleInRadians();
        return distance * Math.cos(gyroAngle);
    }

}
