//TODO - NEED TO SWITCH OUT VERTICAL SERVO. REPLACEMENT IN MIDDLE OF BOT.

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
    ColorSensor sensorColor;

   // GyroHelper gyroHelper;

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
        LeftFeedMotor = hardwareMap.dcMotor.get("leftfeed");
        RightFeedMotor = hardwareMap.dcMotor.get("rightfeed");
        LeftFeedMotor.setDirection(DcMotor.Direction.REVERSE);
        pickupArmHelper = new PickupArmHelper(telemetry, hardwareMap);
        pickupArmHelper.init();
        sensorColor = hardwareMap.get(ColorSensor.class, "colorsensor");
     //   gyroHelper = new GyroHelper(telemetry, hardwareMap);
       // gyroHelper.moveHelper = moveHelper;
        //gyroHelper.init();

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
        //gyroHelper.init_loop();
    }

    @Override
    public void loop() {
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

        if (gamepad1.b) {
        blockArmServoHelper.Open();

        }
        if (gamepad1.x) {
        blockArmServoHelper.Close();
        }

//        telemetry.addData("BlockArmPosition", blockArmPosition);
        /*
        telemetry.addData("ElevationArm", pickupArmHelper.getPosition());
        if (sensorColor != null) {
            telemetry.addData("red: ", sensorColor.red());
            telemetry.addData("blue: ", sensorColor.blue());
            telemetry.addData("green: ", sensorColor.green());
        } else {
            telemetry.addData("Color sensor is disabled","");
        }*/

       // gyroHelper.printHeadings();
      //  telemetry.update();

    }
}
