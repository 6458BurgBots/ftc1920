package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helper.BlockArmServoHelper;
import org.firstinspires.ftc.teamcode.Helper.BuildPlateServoHelper;
import org.firstinspires.ftc.teamcode.Helper.IMUHelper;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;
import org.firstinspires.ftc.teamcode.Helper.ClawHelper;
import org.firstinspires.ftc.teamcode.Helper.PickupArmHelper;

@TeleOp(name="TeleOpNew", group="TeleOp")
public class TeleOpNew extends OpMode{

    MoveHelper moveHelper;
    BlockArmServoHelper blockArmServoHelper;
    private DcMotor LeftFeedMotor;
    private DcMotor RightFeedMotor;
    ClawHelper pickupArmHelper;
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
        pickupArmHelper = new ClawHelper(telemetry, hardwareMap);
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
        moveHelper.checkTeleOp(gamepad1,gamepad2);
        buildPlateServoHelper.checkTeleOp(gamepad1, gamepad2);
        pickupArmHelper.checkTeleOp(gamepad1, gamepad2);
        //imuHelper.checkTeleOp(gamepad1, gamepad2);
        double LY = gamepad1.left_trigger/2 -gamepad1.right_trigger/2;

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
