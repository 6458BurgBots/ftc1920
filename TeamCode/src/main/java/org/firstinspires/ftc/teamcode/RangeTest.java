package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="RangeTest", group="TeleOp")
public class RangeTest extends LinearOpMode{
//    AnalogInput MRLightSensor;
    //CDI. Using this, we can read any analog sensor on this CDI without creating an instance for each sensor.
//    DeviceInterfaceModule opticalDistanceSensor;
    DistanceSensor sensorRange;

    private ElapsedTime runtime = new ElapsedTime();

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

//    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
//    public static final int RANGE1_REG_START = 0x04; //Register to start reading
//    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
//    public I2cDevice RANGE1;
//    public I2cDeviceSynch RANGE1Reader;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        MRLightSensor = hardwareMap.analogInput.get("light");
//        opticalDistanceSensor = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        sensorRange = hardwareMap.get(DistanceSensor.class, "range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
//        RANGE1 = hardwareMap.i2cDevice.get("range");
//        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
//        RANGE1Reader.engage();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
//            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            //Read the light sensor using the Analog Input object
//            telemetry.addData("light", MRLightSensor.getVoltage());
            telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
//            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
//            telemetry.addData("ODS", range1Cache[1] & 0xFF);
//            telemetry.addData("Status", "Run Time: " + runtime.toString());


            //Read each Analog Port of the CDI. 0-7
            for (int i = 0; i < 8; i++) {
//                telemetry.addData("Analog " + i, opticalDistanceSensor.getAnalogInputVoltage(i));
            }
            telemetry.update();
            idle();
        }
    }
}
