package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.IMUHelper;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@TeleOp(name="TestColorSensor", group="TeleOp")
public class TestColorSensor extends OpMode {
    MoveHelper moveHelper;
    ColorSensor blockColor;

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        moveHelper.resetEncoders();
        moveHelper.runWithoutEncoders();
        moveHelper.displayMoveOutputs = false;
        moveHelper.runWithoutEncoders();
        blockColor = hardwareMap.get(ColorSensor.class,"blockdetector");
    }

    @Override
    public void loop() {
        telemetry.addData("IsSkyStone", isSkyStone(blockColor));
        telemetry.addData("IsCloseToBlock", isCloseToBlock(blockColor));
        telemetry.update();
    }

    private boolean isSkyStone(ColorSensor blockColor){
        double redToBlue = blockColor.red() / blockColor.blue();
        double greenToBlue = blockColor.green() / blockColor.blue();
        if (isCloseToBlock(blockColor)) {

            if (redToBlue < 2 && greenToBlue < 3) {
                return true;
            } else {
                return false;
            }
        }
        telemetry.addData("redToBlue", redToBlue);
        telemetry.addData("greenToBlue", greenToBlue);
        return false;
    }

    private boolean isCloseToBlock(ColorSensor blockColor){
        double totalBlockSensorValues = blockColor.red() + blockColor.blue() + blockColor.green();
        telemetry.addData("totalBlockSensorValues", totalBlockSensorValues);
        if (totalBlockSensorValues < 1500){
            return false;
        }
        return true;
    }
}
