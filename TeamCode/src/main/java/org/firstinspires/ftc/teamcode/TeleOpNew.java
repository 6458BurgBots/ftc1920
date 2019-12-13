package org.firstinspires.ftc.teamcode;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.BlockArmServoHelper;
import org.firstinspires.ftc.teamcode.Helper.BuildPlateServoHelper;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;
import org.firstinspires.ftc.teamcode.Helper.PickupArmHelper;

@TeleOp(name="TeleOpNew", group="TeleOp")
public class TeleOpNew extends OpMode{

    MoveHelper moveHelper;
    BlockArmServoHelper blockArmServoHelper;
    private DcMotor LeftFeedMotor;
    private DcMotor RightFeedMotor;
    double blockArmPosition = 0;
    PickupArmHelper pickupArmHelper;
    BuildPlateServoHelper buildPlateServoHelper;
   // public static double BLOCK_ARM_SERVO_CLOSED = 1;      //Fix later
   // public static double BLOCK_ARM_SERVO_OPEN = 0.5;

    @Override
    public void init() {

        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();
        blockArmServoHelper = new BlockArmServoHelper(telemetry, hardwareMap);
        blockArmServoHelper.init();
        moveHelper.resetEncoders();
        moveHelper.runUsingEncoders();
        buildPlateServoHelper = new BuildPlateServoHelper(telemetry, hardwareMap);
        buildPlateServoHelper.init();
        //moveHelper.runWithoutEncoders();
        LeftFeedMotor = hardwareMap.dcMotor.get("leftfeed");
        RightFeedMotor = hardwareMap.dcMotor.get("rightfeed");
        LeftFeedMotor.setDirection(DcMotor.Direction.REVERSE);
        blockArmServoHelper.GoToPosition(blockArmPosition);
        pickupArmHelper = new PickupArmHelper(telemetry, hardwareMap);
        pickupArmHelper.init();

    }

    @Override
    public void loop() {
        telemetry.addData("BlockArmPosition",blockArmPosition);
        moveHelper.checkTeleOp(gamepad1,gamepad2);
        buildPlateServoHelper.checkTeleOp(gamepad1, gamepad2);
        pickupArmHelper.checkTeleOp(gamepad1, gamepad2);
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

//        if (gamepad1.b) {
   //     blockArmServoHelper.GoToPosition(-1);

//        }
//        if (gamepad1.x) {
//        blockArmServoHelper.GoToPosition(-.5);

//        }

        if (gamepad1.x) {
            blockArmPosition += 1;

        }
//        if (gamepad1.a) {
//            blockArmPosition = .5;      //change to b if it works
//        }
        if (gamepad1.b) {
            blockArmPosition -= 1;

        }
        if (gamepad2.left_stick_y != 0){
            pickupArmHelper.raise(gamepad2.left_stick_y);       //Could be negative
        }

        blockArmServoHelper.GoToPosition(blockArmPosition);
    }
}
