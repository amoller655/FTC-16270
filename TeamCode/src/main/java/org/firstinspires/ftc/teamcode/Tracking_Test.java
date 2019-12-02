package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;


@TeleOp(name="Tracking Test", group="test")
public class Tracking_Test extends OpMode {

    private DriveTrain driveTrain;

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightRear;
    private DcMotor leftRear;

    private BNO055IMU imu;

    private ElapsedTime runtime;

    private int state;


    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftRear = hardwareMap.dcMotor.get("leftRear");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        driveTrain = new DriveTrain(leftFront, rightFront, leftRear, rightRear, imu, telemetry);
        state = 1;

        runtime = new ElapsedTime();
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
                if(driveTrain.getYPos() > -6.0)
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
