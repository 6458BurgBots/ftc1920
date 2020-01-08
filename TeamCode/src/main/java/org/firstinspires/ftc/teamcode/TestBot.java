//Old

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Helper.MoveHelper;

@Disabled
@TeleOp(name="TeleOp", group="TeleOp")
public class TestBot extends OpMode{

    MoveHelper moveHelper;

    @Override
    public void init() {
        moveHelper = new MoveHelper(telemetry, hardwareMap);
        moveHelper.init();

    }

    @Override
    public void loop() {
        moveHelper.checkTeleOp(gamepad1,gamepad2);
    }
}
