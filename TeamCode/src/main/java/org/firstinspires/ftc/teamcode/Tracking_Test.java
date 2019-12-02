package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;


@TeleOp(name="Tracking Test", group="test")
public class Tracking_Test extends OpMode {

    private DriveTrain driveTrain;

    private BNO055IMU imu;



    @Override
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        driveTrain = new DriveTrain(imu, telemetry);
    }

    @Override
    public void loop() {

        telemetry.addLine("Right Stick!")
                .addData("X", gamepad1.right_stick_x)
                .addData("Y", gamepad1.right_stick_y);
        telemetry.addLine("Right Trigger!")
                .addData("X", gamepad1.left_trigger)
                .addData("Y", gamepad1.right_trigger);
        telemetry.addLine("Position")
                .addData("X", driveTrain.getXPos())
                .addData("Y", driveTrain.getYPos())
                .addData("Z", driveTrain.getZPos());
        telemetry.addLine("Velocity")
                .addData("X", driveTrain.getXVel())
                .addData("Y", driveTrain.getYVel())
                .addData("Z", driveTrain.getZVel());
        telemetry.addLine("Acceleration")
                .addData("X", driveTrain.getXAcc())
                .addData("Y", driveTrain.getYAcc())
                .addData("Z", driveTrain.getZAcc());
        telemetry.update();
    }
}
