package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp(name = "Odometry System Calibration", group = "util")
public class OdometryCalibration extends LinearOpMode {
//    Motors
    DcMotor leftFront, rightFront, leftRear, rightRear;
//    Odometry wheels
    DcMotor verticalLeft, verticalRight, horizontal;

//    IMU
    BNO055IMU imu;

//    hardware map names
    String lfName = "leftFront", rfName = "rightFront", lrName = "leftRear", rrName = "rightRear";
    String verticalLeftName = lfName, verticalRightName = rfName, horizonatalName = lrName;

//    encoder counts per inch robot moved NEEDS TO BE UPDATED
    final double COUNTS_PER_INCH = 300.0;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    //    Files for values to be written to. Stored under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.dcMotor.get(lfName);
        rightFront = hardwareMap.dcMotor.get(rfName);
        leftRear = hardwareMap.dcMotor.get(lrName);
        rightRear = hardwareMap.dcMotor.get(rrName);

        verticalLeft = hardwareMap.dcMotor.get(verticalLeftName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightName);
        horizontal = hardwareMap.dcMotor.get(horizonatalName);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}
