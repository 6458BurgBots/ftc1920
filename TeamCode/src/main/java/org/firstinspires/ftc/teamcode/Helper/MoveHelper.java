package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by WilliamsburgRobotic on 10/31/2017.
 */

public class MoveHelper extends OperationHelper {

    private double powerMultiple = .75;

    private static final double ENCODER_POWER_LEVEL = 1;
    // declares the motors; gives them names we will use to call them later
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private boolean isPositionValid;
    public double encoderPowerLevel = 1;
    public boolean joystickJacob = true;
    public boolean displayMoveOutputs = true;


    public MoveHelper(Telemetry t, HardwareMap h)
    {
        super(t, h);
    }

    public void init( ) {
        FLMotor = hardwareMap.dcMotor.get("FL");
        FRMotor = hardwareMap.dcMotor.get("FR");
        BLMotor = hardwareMap.dcMotor.get("BL");
        BRMotor = hardwareMap.dcMotor.get("BR");
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void SetHighPower() {
        powerMultiple = .75;
    }

    public void SetLowPower() {
        powerMultiple =.33;
    }

    public void omniDrive(double lx,double ly, double rx){
        telemetry.addData("Drive input (lx,ly): ", lx + "," + ly);
        // omni-drive math, sets it up to run properly
        double fl = ly - lx - rx;
        double fr = ly + lx + rx;
        double bl = ly + lx - rx;
        double br = ly - lx + rx;

        double max = Math.max(Math.max(fl, fr), Math.max(bl, br));
        if (max > 1) {
            fl = fl / max;
            fr = fr / max;
            bl = bl / max;
            br = br / max;
        }

        String output = String.format("%1$.3f,%1$.3f,%1$.3f,%1$.3f",fl,fr,bl,br);
        telemetry.addData("Driving (fl,fr,bl,br): ", output);
        // sets power to the motors
        FLMotor.setPower(fl);
        FRMotor.setPower(fr);
        BLMotor.setPower(bl*.85);
        BRMotor.setPower(br*.85);

    }

    public void rturn(double rx){
        // method used to turn the robot
        FLMotor.setPower(rx);
        FRMotor.setPower(-rx);
        BLMotor.setPower(rx);
        BRMotor.setPower(-rx);
    }
    public void lturn(double lx){
        // method used to turn the robot
        FLMotor.setPower(-lx);
        FRMotor.setPower(lx);
        BLMotor.setPower(-lx);
        BRMotor.setPower(lx);
    }


    // actually turns on the motors/sets power??
    public void runFLMotor (double power){
        FLMotor.setPower(power);
    }
    public void runFRMotor (double power){
        FRMotor.setPower(power);
    }
    public void runBLMotor (double power){
        BLMotor.setPower(power);
    }
    public void runBRMotor (double power){
        BRMotor.setPower(power);
    }

    public void driveForward (double power){
        telemetry.addData("moving forward", power);
        FLMotor.setPower(power);
        FRMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);
    }



    public void loop(){
    }
    public void showEncoderValues (){
        telemetry.addData("BR Encoder", BRMotor.getCurrentPosition());
        telemetry.addData("BL Encoder", BLMotor.getCurrentPosition());
        telemetry.addData("FR Encoder", FRMotor.getCurrentPosition());
        telemetry.addData("FL Encoder", FLMotor.getCurrentPosition());

    }
    public void resetEncoders (){
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        isPositionValid = false;
    }
    public void runUsingEncoders (){
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoders(){
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveToPosition (int position){
        FLMotor.setTargetPosition(position);
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getEncoderValue(){
        return FLMotor.getCurrentPosition();

    }

    public void runOneMotor(DcMotor motor, int position){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(encoderPowerLevel);
    }

    public void continueOneMotor(DcMotor motor){
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(encoderPowerLevel);
        telemetry.addData("Continue target: " + motor.getDeviceName(),motor.getTargetPosition());
    }

    public void runMotorsToPosition(int flPos, int frPos, int brPos, int blPos){
        if (!isPositionValid) {
            runOneMotor(FLMotor, flPos);
            runOneMotor(FRMotor, frPos);
            runOneMotor(BRMotor, brPos);
            runOneMotor(BLMotor, blPos);
            isPositionValid = true;
        }
    }
    public boolean areMotorsBusy() {
        return FLMotor.isBusy() || FRMotor.isBusy() || BRMotor.isBusy() || BLMotor.isBusy();
    }

    public void continueToPosition(){
        if (isPositionValid) {
            continueOneMotor(FLMotor);
            continueOneMotor(FRMotor);
            continueOneMotor(BRMotor);
            continueOneMotor(BLMotor);
        }
    }

    public void checkTeleOp(Gamepad gamepad1,Gamepad gamepad2){
        // alaina is struggling to find a way to describe this
        double LY;
        double LX;
        double RX;

         // Jacob wants the sticks reversed for driving.
        if (joystickJacob) {
            LY = gamepad1.left_stick_y*powerMultiple;
            LX = gamepad1.left_stick_x*powerMultiple;
            RX = gamepad1.right_stick_x*powerMultiple;
        } else {
            LY = gamepad1.right_stick_y*powerMultiple;
            LX = gamepad1.right_stick_x*powerMultiple;
            RX = gamepad1.left_stick_x*powerMultiple;
        }

        if (gamepad1.a) {
            SetLowPower();
        } else if (gamepad1.y) {
            SetHighPower();
        }

        if (gamepad1.dpad_left) {
            runUsingEncoders();
        }
        if (gamepad1.dpad_right) {
            runWithoutEncoders();
        }


        //Establishes floating variables linked to the gamepads
        if (displayMoveOutputs) {
            telemetry.addData("Left X", LX);
            telemetry.addData("Left Y", LY);
            telemetry.addData("Right X", RX);
            telemetry.addData("BR Encoder", BRMotor.getCurrentPosition());
            telemetry.addData("BL Encoder", BLMotor.getCurrentPosition());
            telemetry.addData("FR Encoder", FRMotor.getCurrentPosition());
            telemetry.addData("FL Encoder", FLMotor.getCurrentPosition());
        }

        LY = Range.clip(LY, -1, 1);
        LX = Range.clip(LX, -1, 1);
        RX = Range.clip(RX, -1, 1);

        omniDrive(LX, LY, RX);
/*
        if (gamepad1.a) {
            BLMotor.setPower(.3);
        }
        if (gamepad1.b) {
            BRMotor.setPower(.3);
        }
        if (gamepad1.x) {
            FLMotor.setPower(.3);
        }
        if (gamepad1.y) {
            FRMotor.setPower(.3);
        }
*/

    }
    public int GetBRMotorPosition(){
        return BRMotor.getCurrentPosition();
    }
    public int GetBLMotorPosition(){
        return BLMotor.getCurrentPosition();
    }
    public int GetFRMotorPosition(){
        return FRMotor.getCurrentPosition();
    }
    public int GetFLMotorPosition(){
        return FLMotor.getCurrentPosition();
    }

}
