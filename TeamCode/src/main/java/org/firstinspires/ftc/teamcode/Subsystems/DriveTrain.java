
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class DriveTrain{

    private int leftPos;
    private int rightPos;
    private double avgEncoderPos;

    private double startPos;
    private double goalPos;

    private final int SHAFT_CPR = 1440;
    private final int WHEEL_CPR = 720;
    private final int DIAMETER = 4;
    private final double CIRCUMFERENCE = DIAMETER * Math.PI;
    private final double INPERPULSE = CIRCUMFERENCE / WHEEL_CPR;
    private final double PULSEPERIN = WHEEL_CPR / CIRCUMFERENCE;

    private float goalDegrees;
    private int GYRO_RANGE = 3;


    private boolean canDrive;




//    MOTORS
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

//    IMU
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;

//    TELEMETRY
    private Telemetry telemetry;

//    CONSTRUCTOR (FULL)
    public DriveTrain(DcMotor leftFront, DcMotor rightFront, DcMotor leftRear, DcMotor rightRear, BNO055IMU imu, Telemetry telemetry){
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;

        this.imu = imu;

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "Hub1Alone.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu.initialize(parameters);



        this.telemetry = telemetry;

        this.telemetry.addData("DriveTrain.java Startup ", "Initiating");
        this.telemetry.update();

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);

        canDrive = true;

    }

//    CONSTRUCTOR (IMU ONLY)



        telemetry.addData("DriveTrain.java Startup ", "Completed");

        startPos = -1;
        goalPos = startPos;
        goalDegrees = -1;
    }

    public DriveTrain(BNO055IMU imu, Telemetry telemetry)
    {
        this.imu = imu;

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "Hub1Alone.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu.initialize(parameters);
    }

    public void getHeading(){

    }

    public void setMotorPower(double x, double y, double z){

        /*
        Guide to motor Powers:
        Left Front: - (y + x + z)
        Right Front: y - x - z
        Left Rear:  - (y - x + z)
        Right Rear:  y + x - z
         */
        this.leftFront.setPower(Range.clip((y+x+z),-1,1));
        this.rightFront.setPower(Range.clip((y-x-z),-1,1));
        this.leftRear.setPower(Range.clip((y-x+z),-1,1));
        this.rightRear.setPower(Range.clip((y+x-z), -1, 1));
    }

    public void drive(Direction direction, double power){
        switch(direction){
            case N:
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftRear.setPower(power);
                rightRear.setPower(power);
                break;
            case S:
                leftFront.setPower(-power);
                rightFront.setPower(-power);
                leftRear.setPower(-power);
                rightRear.setPower(-power);
                break;
            case W:
                leftFront.setPower(-power);
                leftRear.setPower(power);
                rightFront.setPower(power);
                rightRear.setPower(-power);
                break;
            case E:
                leftFront.setPower(power);
                leftRear.setPower(-power);
                rightFront.setPower(-power);
                rightRear.setPower(power);
                break;
            case TURNLEFT:
                leftFront.setPower(-power);
                leftRear.setPower(-power);
                rightFront.setPower(power);
                rightRear.setPower(power);
                break;
            case TURNRIGHT:
                leftFront.setPower(power);
                leftRear.setPower(power);
                rightFront.setPower(-power);
                rightRear.setPower(-power);
                break;
            case NW:
                leftFront.setPower(power);
                leftRear.setPower(0.0);
                rightFront.setPower(0.0);
                rightRear.setPower(power);
                break;
            case NE:
                leftFront.setPower(0.0);
                leftRear.setPower(power);
                rightFront.setPower(power);
                rightRear.setPower(0.0);
                break;
            case SW:
                leftFront.setPower(0.0);
                leftRear.setPower(-power);
                rightFront.setPower(-power);
                rightRear.setPower(0.0);
                break;
            case SE:
                leftFront.setPower(-power);
                leftRear.setPower(0.0);
                rightFront.setPower(0.0);
                rightRear.setPower(-power);
                break;
            case ENE:
                leftFront.setPower(power);
                leftRear.setPower(-power/2);
                rightFront.setPower(-power/2);
                rightRear.setPower(power);
                break;
            case NNE:
                leftFront.setPower(power);
                rightFront.setPower(power/2);
                leftRear.setPower(power/2);
                rightRear.setPower(power);
                break;
            case NNW:
                leftFront.setPower(power/2);
                rightFront.setPower(power);
                leftRear.setPower(power);
                rightRear.setPower(power/2);
                break;
            case WNW:
                leftFront.setPower(-power/2);
                rightFront.setPower(power);
                leftRear.setPower(power);
                rightRear.setPower(-power);
                break;
            case WSW:
                leftFront.setPower(-power);
                rightFront.setPower(power/2);
                leftRear.setPower(power/2);
                rightRear.setPower(-power/2);
                break;
        }
    }

    public void stop(){
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
    }

    public void resetEncoders()
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean encoderDrive(Direction direction, double inches, double power)
    {
        leftPos = leftFront.getCurrentPosition();
        rightPos = rightFront.getCurrentPosition();
        avgEncoderPos = (double) (leftPos + rightPos) / 2;
        if(canDrive)
        {

            goalPos = (inches * PULSEPERIN) + avgEncoderPos;
            canDrive = false;
            return canDrive;
        }
        switch(direction)
        {
            case N:
                if(avgEncoderPos < goalPos)
                {
                    drive(direction, power);
                    telemetry.addLine()
                            .addData("Current pos: ", avgEncoderPos)
                            .addData(" Goal Pos: ", goalPos);
                } else {
                    canDrive = true;
                    goalPos = startPos;
                    stop();
                    return canDrive;
                }
                break;
            case S:
                if(avgEncoderPos > goalPos)
                {
                    drive(direction, power);
                    telemetry.addLine()
                            .addData("Current pos: ", avgEncoderPos)
                            .addData(" Goal Pos: ", goalPos);
                } else {
                    canDrive = true;
                    goalPos = startPos;
                    stop();
                    return canDrive;
                }
                break;
        }
        return canDrive;
    }

    public boolean gyroTurn(Direction direction, double power, float degrees)
    {
        switch(direction)
        {
            case TURNLEFT:
                if (goalDegrees == -1)
                {
                    goalDegrees = (this.getYaw() + degrees);
                    if(goalDegrees > 360) goalDegrees -= 360;
                }
                if(!(getYaw() < (goalDegrees + GYRO_RANGE) && (getYaw() > (goalDegrees - GYRO_RANGE))))
                {
                    drive(direction, power);

                    telemetry.addLine()
                            .addData("Goal degrees: ", goalDegrees)
                            .addData(" Current degrees: ", getYaw());
                    return false;
                } else {
                    stop();
                    goalDegrees = -1;
                    return true;
                }

            case TURNRIGHT:
                if (goalDegrees == -1)
                {
                    goalDegrees = (this.getYaw() - degrees);
                    if(goalDegrees < 0) goalDegrees += 360;
                }
                if(!(getYaw() < (goalDegrees + GYRO_RANGE) && (getYaw() > (goalDegrees - GYRO_RANGE))))
                {
                    drive(direction, power);

                    telemetry.addLine()
                            .addData("Goal degrees: ", goalDegrees)
                            .addData(" Current degrees: ", getYaw());
                    return false;
                } else {
                    stop();
                    goalDegrees = -1;
                    return true;
                }

        }
        return false;
    }

    public float getYaw()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < 0)
        {
            return (360 + AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        } else {
            return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        }
    }



    public enum Direction{
        N,
        S,
        E,
        W,
        TURNRIGHT,
        TURNLEFT,
        NE,
        SE,
        NW,
        SW,
        ENE,
        NNE,
        NNW,
        WNW,
        WSW,
        SSW,
        SSE,
        ESE
    }
}