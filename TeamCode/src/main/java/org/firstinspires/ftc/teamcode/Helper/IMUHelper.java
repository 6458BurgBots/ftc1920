package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;
import static java.lang.Math.round;

/**
 * Created by WilliamsburgRobotic on 10/31/2017.
 */

public class IMUHelper extends OperationHelper {

    private static double ANGLE_TOLERANCE = 5;
    BNO055IMU imu;
    private boolean initializationStarted=false;
    private boolean angleReset = false;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;
    double                  desiredAngle = 0;

    public MoveHelper moveHelper;

    public IMUHelper(Telemetry t, HardwareMap h) {
        super(t, h);
    }

    @Override
    public void init() {
        telemetry.addData("Before IMU INIT", hardwareMap);
        super.init();

        if (hardwareMap != null) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
        } else {
            telemetry.addLine("HardwareMap is null");
        }
        telemetry.update();
    }

    public void init_loop(){
        if (!initializationStarted) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            initializationStarted = true;
            parameters.mode                = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = false;
            imu.initialize(parameters);
        }
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
    }

    public void printHeadings() {
        telemetry.addData("Heading 1", lastAngles.firstAngle);
        //telemetry.addData("Heading 2", lastAngles.secondAngle);
        //telemetry.addData("Heading 3", lastAngles.thirdAngle);
        telemetry.addData("global heading", globalAngle);
    }
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
        desiredAngle = 0;
    }

    public double getAngleInRadians (){
        return getAngle() * Math.PI / 180;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {

        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle = (globalAngle - deltaAngle) % 360;

        lastAngles = angles;

        return globalAngle;
    }

    public void checkTeleOp(Gamepad gamepad1, Gamepad gamepad2){
        if (!angleReset) {
            resetAngle();
            angleReset = true;
        }
        if (gamepad1.dpad_left) {
            desiredAngle = (90) % 360;
        }
        if (gamepad1.dpad_right) {
            desiredAngle = (90) % 360;
        }
        getAngle();
        if (abs(desiredAngle - globalAngle) > 5) {
            turnTo((int)round(desiredAngle));
        } else {
            moveHelper.rturn(0);
            telemetry.addData("Trying to stop","");
        }
    }

    public void turnTo(int desired) {
        double TURN_RIGHT_SPEED = -.25;
        double TURN_LEFT_SPEED = .25;

        getAngle();

        double direction = desired - globalAngle;
        double magnitude = abs(direction);

        telemetry.addData("Direction", direction);
        telemetry.addData("Desired", desired);
        printHeadings();

        if (magnitude < ANGLE_TOLERANCE) {
            telemetry.addData("Turn", "right");
        }
        if (direction >= 0 && magnitude <= 180) {
            moveHelper.rturn(TURN_RIGHT_SPEED);
            telemetry.addData("Turn", "right");
        } else if (direction < 0 && magnitude > 180) {
            moveHelper.rturn(TURN_RIGHT_SPEED);
            telemetry.addData("Turn", "right");
        } else if (direction >= 0 && magnitude >= 180) {
            moveHelper.rturn(TURN_LEFT_SPEED);
            telemetry.addData("Turn", "left");
        } else if (direction < 0 && magnitude < 180) {
            moveHelper.rturn(TURN_LEFT_SPEED);
            telemetry.addData("Turn", "left");
        }
    }
}