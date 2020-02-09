package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

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

    final double PIVOT_SPEED = 0.5;

//    encoder counts per inch robot moved NEEDS TO BE UPDATED
    final double COUNTS_PER_INCH = 288.8665556;

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

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status: ", "Hardware Map Init Complete");
        telemetry.update();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

//        Begin calibration (if robot is unable to pivot at theses speeds, please adjust PIVOT_SPEED above)
        while(getZAngle() < 90 && opModeIsActive()){
            setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
            if(getZAngle() < 60) {
                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
            } else {
                setPowerAll(-PIVOT_SPEED/2, -PIVOT_SPEED/2, PIVOT_SPEED/2, PIVOT_SPEED/2);
            }

            telemetry.addData("IMU Angle: ", getZAngle());
            telemetry.update();
        }

//        Stop the robot
        setPowerAll(0,0,0,0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()) {
            telemetry.addData("IMU Angle: ", getZAngle());
            telemetry.update();
        }

//        Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getZAngle();

        /*
        * Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        * Since the left encoder is also mapped to a drive motor, the encoder value nees to be reversed with the negative sign in front
        * THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
        * */
        double encoderDifference = Math.abs(verticalLeft.getCurrentPosition()) + (Math.abs(verticalRight.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        horizontalTickOffset = horizontal.getCurrentPosition()/Math.toRadians(getZAngle());

        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive())
        {
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", -verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }

    private double getZAngle() {
        return (-imu.getAngularOrientation().firstAngle);
    }
    private void setPowerAll(double rf, double rb, double lf, double lb){
        rightFront.setPower(rf);
        rightRear.setPower(rb);
        leftFront.setPower(lf);
        leftRear.setPower(lb);
    }
}
