package org.firstinspires.ftc.teamcode;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.BlockArmServoHelper;
import org.firstinspires.ftc.teamcode.Helper.BuildPlateServoHelper;
import org.firstinspires.ftc.teamcode.Helper.GyroHelper;
import org.firstinspires.ftc.teamcode.Helper.IMUHelper;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;
import org.firstinspires.ftc.teamcode.Helper.PickupArmHelper;
import org.firstinspires.ftc.teamcode.Helper.PickupArmHelper2;

@TeleOp(name="TeleOpNew", group="TeleOp")
public class TeleOpNew extends OpMode{

    MoveHelper moveHelper;
    BlockArmServoHelper blockArmServoHelper;
    private DcMotor LeftFeedMotor;
    private DcMotor RightFeedMotor;
    double blockArmPosition = 0;
    PickupArmHelper2 pickupArmHelper;
    BuildPlateServoHelper buildPlateServoHelper;
    ColorSensor sensorColor;
    ColorSensor blockColor;
    IMUHelper imuHelper;

    @Override
    public void init() {

        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        moveHelper.resetEncoders();
        moveHelper.runUsingEncoders();
        moveHelper.displayMoveOutputs = false;
        blockArmServoHelper = new BlockArmServoHelper(telemetry, hardwareMap);
        blockArmServoHelper.init();
        pickupArmHelper = new PickupArmHelper2(telemetry, hardwareMap);
        pickupArmHelper.init();
        buildPlateServoHelper = new BuildPlateServoHelper(telemetry, hardwareMap);
        buildPlateServoHelper.init();
        LeftFeedMotor = hardwareMap.dcMotor.get("leftfeed");
        RightFeedMotor = hardwareMap.dcMotor.get("rightfeed");
        LeftFeedMotor.setDirection(DcMotor.Direction.REVERSE);
        sensorColor = hardwareMap.get(ColorSensor.class, "colorsensor");
        blockColor = hardwareMap.get(ColorSensor.class,"blockdetector");
        imuHelper = new IMUHelper(telemetry, hardwareMap);
        imuHelper.moveHelper = moveHelper;
        imuHelper.init();
    }

    public void init_loop() {
        if (moveHelper == null){
            moveHelper = new MoveHelper(telemetry, hardwareMap);
        }
        if (gamepad1.a){ // Left stick is cardinal moves, rotation on right
            moveHelper.joystickJacob = true;
            telemetry.addData("Joystick setup", " Jacob's Way");
        }
        if (gamepad1.y) { // Left stick is rotation, cardinal moves on right
            moveHelper.joystickJacob = false;
            telemetry.addData("Joystick setup", " Thomas' Way");
        }
        imuHelper.init_loop();
    }

    @Override
    public void loop() {
        //moveHelper.checkTeleOp(gamepad1,gamepad2);
        buildPlateServoHelper.checkTeleOp(gamepad1, gamepad2);
        pickupArmHelper.checkTeleOp(gamepad1, gamepad2);
        imuHelper.checkTeleOp(gamepad1, gamepad2);
        double LY = gamepad1.left_trigger/4 -gamepad1.right_trigger/4;

        LeftFeedMotor.setPower(LY);
        RightFeedMotor.setPower(LY);

        if (gamepad1.left_stick_button){        //Resets Encoder Values With Left Stick Button
            moveHelper.resetEncoders();
            moveHelper.runUsingEncoders();
        }
        if (gamepad1.right_stick_button){
            moveHelper.runWithoutEncoders();
        }

        if (gamepad1.b) {
        blockArmServoHelper.Open();

        }
        if (gamepad1.x) {
        blockArmServoHelper.Close();
        }
/*
        if (isSkyStone(blockColor.red(), blockColor.green(), blockColor.blue())){
            telemetry.addData("SkyStone", "yes");
        }else {
            telemetry.addData("SkyStone", "no");
        }

        telemetry.addData("isCloseToBlock", isCloseToBlock(blockColor.red(), blockColor.green(), blockColor.blue()));
*/


//        telemetry.addData("BlockArmPosition", blockArmPosition);

        //telemetry.addData("ElevationArm", pickupArmHelper.getPosition());

//        if (sensorColor != null) {
//            telemetry.addData("red: ", sensorColor.red());
//            telemetry.addData("blue: ", sensorColor.blue());
//            telemetry.addData("green: ", sensorColor.green());
//        } else {
//            telemetry.addData("Color sensor is disabled","");
//        }
/*
        if (blockColor != null) {
            telemetry.addData("red: ", blockColor.red());
            telemetry.addData("blue: ", blockColor.blue());
           telemetry.addData("green: ", blockColor.green());
        } else {
            telemetry.addData("Block sensor is disabled","");
       }
 */
        imuHelper.printHeadings();
        telemetry.update();
    }
    private boolean isSkyStone(int red, int green, int blue){
        double redToBlue = red / blue;
        double greenToBlue = green / blue;
        if (isCloseToBlock(red, green, blue)) {


            if (redToBlue < 2 && greenToBlue < 3) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }
    private boolean isCloseToBlock(int red, int green, int blue){
        double totalBlockSensorValues = red + blue + green;
        if (totalBlockSensorValues < 1500){
            return false;
        }
        return true;
    }
}
