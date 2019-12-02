package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;


@TeleOp(name="Tracking Test", group="test")
public class Tracking_Test extends OpMode {

    private DriveTrain driveTrain;

    private BNO055IMU imu;

    private ElapsedTime runtime;

    private int state;


    @Override
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        driveTrain = new DriveTrain(imu, telemetry);
        state = 1;
    }

    @Override
    public void loop() {

        switch(state)
        {
            case 1:
                runtime.reset();
                state++;
                break;
            case 2:
                if(!driveTrain.integrating()) driveTrain.startIntegration();
                if(runtime.seconds() <= 1.0)
                {
                    driveTrain.drive(DriveTrain.Direction.N, 0.5);
                }
                else {
                    driveTrain.stop();
                    driveTrain.stopIntegration();
                    state++;
                }
                break;
            case 3:
                if(gamepad1.a){
                    driveTrain.startIntegration();
                    state++;
                }
                break;
            case 4:
                if(driveTrain.getXPos() <= 6.0)
                {
                    driveTrain.drive(DriveTrain.Direction.N, 0.5);
                }
                else
                {
                    driveTrain.stop();
                    driveTrain.stopIntegration();
                    state++;
                }
                break;

        }
        telemetry.addLine("Position: ")
                .addData("X: ", driveTrain.getXPos())
                .addData(" Y: ", driveTrain.getYPos())
                .addData(" Z: ", driveTrain.getZPos());
        telemetry.update();
    }
}
