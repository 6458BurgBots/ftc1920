package org.firstinspires.ftc.teamcode.Helper;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.GyroSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.OperationHelper;

import static java.lang.Math.abs;

/**
 * Created by WilliamsburgRobotic on 10/31/2017.
 */

public class GyroHelper extends OperationHelper {

    public GyroSensor gyroBoy;   //Make private Later
    private boolean calibrationStarted=false;

    public MoveHelper moveHelper;

    public GyroHelper(Telemetry t, HardwareMap h) {
        super(t, h);
    }

    @Override
    public void init() {

        telemetry.addData("Before GyroSensor INIT", hardwareMap);
        super.init();
        if (hardwareMap != null) {
            gyroBoy = hardwareMap.gyroSensor.get("gyro");

        } else {
            telemetry.addLine("HardwareMap is null");
        }
        telemetry.update();
    }

    public void init_loop(){
        if (!calibrationStarted){
            gyroBoy.calibrate();
            calibrationStarted=true;

        }
        if (!gyroBoy.isCalibrating()){
            telemetry.addData("done calibrating",":)");
        }
    }

    public void printHeadings() {
        telemetry.addData("Status is ", gyroBoy.status());
        telemetry.addData("heading is", gyroBoy.getHeading());
        telemetry.addData("GyroX is", gyroBoy.rawX());
        telemetry.addData("GyroY is", gyroBoy.rawY());
        telemetry.addData("GyroZ is", gyroBoy.rawZ());
        //telemetry.addData("last heading is", lastHeading);
        //telemetry.update();
    }

    @Override
    public int loop(double runTime, int state){
        printHeadings();
        switch (state) {


        }

        return state;
    }

    public void turnCero(){
        if (gyroBoy.getHeading()<=180){
            moveHelper.rturn(-.3);
        }
        else {
            moveHelper.rturn(.3);
        }
    }

    public void turn270(){
        if (gyroBoy.getHeading()>=90&& gyroBoy.getHeading()<270){
            moveHelper.rturn(.3);
        }
        else {
            moveHelper.rturn(-.3);
        }
    }

    public boolean isFacing270(){
        if (gyroBoy.getHeading()>265&&gyroBoy.getHeading()<275){
            return true;
        }
        return false;
    }

    public boolean isFacingCero() {
        if (gyroBoy.getHeading()<5){
            return true;
        }
        if (gyroBoy.getHeading()>355){
            return true;
        }
        return false;
    }

    public void turnTo(int desired) {
        double TURN_RIGHT_SPEED = -.3;
        double TURN_LEFT_SPEED = .3;
        int currentHeading = gyroBoy.getHeading();

        int direction = desired - currentHeading;
        int magnitude = abs(direction);

        if (direction >= 0 && magnitude <= 180) {
            moveHelper.rturn(TURN_LEFT_SPEED);
        } else if (direction < 0 && magnitude > 180) {
            moveHelper.rturn(TURN_LEFT_SPEED);
        } else if (direction >= 0 && magnitude >= 180) {
            moveHelper.rturn(TURN_RIGHT_SPEED);
        } else if (direction < 0 && magnitude < 180) {
            moveHelper.rturn(TURN_RIGHT_SPEED);
        }
    }

    public boolean isFacing(int desired) {
        int currentHeading = gyroBoy.getHeading();

        telemetry.addData("heading", currentHeading);
        telemetry.addData("desired", desired);

        int difference = abs(currentHeading - desired);
        if (difference <= 5) {
            return true;
        }
        if (difference >= 355) {
            return true;
        }
        return false;
    }

}