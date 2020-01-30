package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    public DriveTrain driveTrain;

    public Odometry odometry;

    private DcMotor leftFront, rightFront, leftRear, rightRear;

    private DcMotor leftEncoder, rightEncoder, normalEncoder;

    private String leftFrontName, rightFrontName, leftRearName, rightRearName;
//    CHECK PORT NAMES
    private String leftEncoderName = leftFrontName, rightEncoderName = rightFrontName, normalEncoderName = rightRearName;

//    OBSOLETE: phase out
    private HardwareMap hardwareMap;
    private BNO055IMU imu;
    private String imuName;

    public Robot(HardwareMap hardwareMap, String leftFrontName, String rightFrontName, String leftRearName, String rightRearName, String imuName, Telemetry telemetry) {
        this.leftFrontName = leftFrontName;
        this.rightFrontName = rightFrontName;
        this.leftRearName = leftRearName;
        this.rightRearName = rightRearName;
        this.imuName = imuName;
        this.hardwareMap = hardwareMap;

        initHardwareMap(this.leftFrontName, this.rightFrontName, this.leftRearName, this.rightRearName, this.leftEncoderName, this.rightEncoderName, this.normalEncoderName, this.imuName);

        driveTrain = new DriveTrain(leftFront, rightFront, leftRear, rightRear, imu, telemetry);

        odometry = new Odometry(leftEncoder, rightEncoder, normalEncoder, 0.0, 50);
    }

    private void initHardwareMap(String leftFrontName, String rightFrontName, String leftRearName, String rightRearName, String leftEncoderName, String rightEncoderName, String normalEncoderName, String imuName)
    {
        leftFront = hardwareMap.dcMotor.get(leftFrontName);
        rightFront = hardwareMap.dcMotor.get(rightFrontName);
        leftRear = hardwareMap.dcMotor.get(leftRearName);
        rightRear = hardwareMap.dcMotor.get(rightRearName);

        leftEncoder = hardwareMap.dcMotor.get(leftEncoderName);
        rightEncoder = hardwareMap.dcMotor.get(rightEncoderName);
        normalEncoder = hardwareMap.dcMotor.get(normalEncoderName);

        imu = hardwareMap.get(BNO055IMU.class, imuName);

    }

    public boolean goTo(double xF, double yF, double power, DriveTrain.Direction direction)
    {
        double x = odometry.getXPos();
        double y = odometry.getYPos();
        double theta = odometry.getOrientation();

        double dx = xF - x;
        double dy = yF - y;

        double pathAngle = Math.toDegrees(calculatePathAngle(dx, dy));
        double facingOffset = 0.0;
        switch(direction)
        {
            case N:
                facingOffset = 0;
                break;
            case NE:
                facingOffset = 315;
                break;
            case E:
                facingOffset = 270;
                break;
            case SE:
                facingOffset = 225;
                break;
            case S:
                facingOffset = 180;
                break;
            case SW:
                facingOffset = 135;
                break;
            case W:
                facingOffset = 90;
                break;
            case NW:
                facingOffset = 45;
                break;
            default:
                facingOffset = 0;
                break;
        }
        double goalAngle = wrapAngle(pathAngle - facingOffset);
        if(theta != goalAngle)


    }

    private double calculatePathAngle(double dx, double dy)
    {
        if(dy == 0)
        {
            if(dx > 0) return 0;
            else if(dx < 0) return 180;
        }
        else if(dx == 0)
        {
            if(dy > 0) return 90;
            else if(dy < 0) return 270;
        }
        else if (dx > 0) return (360 + Math.atan2(dx, dy)) % 360;
        else if (dx < 0) return (180 + Math.atan2(dx, dy));
        return 0;
    }

    private double wrapAngle(double angle) {
        if(angle < 0) return angle + 360;
        else if(angle >= 360) return angle - 360;
        else return angle;
    }
}
