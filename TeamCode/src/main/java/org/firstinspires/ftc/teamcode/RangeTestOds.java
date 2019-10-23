package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="RangeTestOds", group="TeleOp")
public class RangeTestOds extends LinearOpMode{

    //An Analog Input. In this example, we used a Light Sensor although it could be any analog sensor.
    AnalogInput MRLightSensor;

    //CDI. Using this, we can read any analog sensor on this CDI without creating an instance for each sensor.
    DeviceInterfaceModule cdi;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Link objects to configuration file
        MRLightSensor = hardwareMap.analogInput.get("light");
        //cdi = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        waitForStart();

        while (opModeIsActive()) {

            //Read the light sensor using the Analog Input object
            telemetry.addData("light", MRLightSensor.getVoltage());

            //Read each Analog Port of the CDI. 0-7
 //           for (int i = 0; i < 8; i++) {
 //               telemetry.addData("Analog " + i, cdi.getAnalogInputVoltage(i));
 //           }
            telemetry.update();
        }
    }
}
