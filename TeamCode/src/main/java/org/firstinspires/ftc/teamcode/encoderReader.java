package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Encoder Tester", group="Test")
public class encoderReader extends OpMode {

    DcMotor encoder;
    @Override
    public void init() {
        encoder = hardwareMap.dcMotor.get("encoder");
    }

    @Override
    public void loop() {
        telemetry.addData("Value: ", encoder.getCurrentPosition());
        telemetry.update();
    }
}
