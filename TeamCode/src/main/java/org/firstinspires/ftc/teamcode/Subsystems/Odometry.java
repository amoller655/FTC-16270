package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class Odometry implements Runnable{
//    Odometry wheels
    private DcMotor verticalEncoderLeft, verticalEncoderRight, horizontalEncoder;

//    Thread run condition
    private boolean isRunning = true;

//    Position variables
    double verticalRightEncoderWheelPos = 0, verticalLeftEncoderWheelPos = 0, normalEncoderWheelPos = 0, changeInRobotOrientation = 0;
    private double robotGlobalXPos = 0, robotGlobalYPos = 0, robotOrientationRad = 0;
    private double lastVerticalRightEncodetWheelPos = 0, lastVerticalLeftEncoderWheelPos = 0, lastNormalEncoderWheelPos = 0;

//    Algorithm constants
    private double robotEncoderWheelDist;
    private double horizontalEncoderTickPerDegreeOffset;

//    Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //    Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private int verticalLeftEncoderPosMultiplier = 1;
    private int verticalRightEncoderPosMultiplier = 1;
    private int normalEncoderPosMultiplier = 1;

    private final double COUNTS_PER_INCH;


    /**
     * Constructor for Odometry Thread
     * @param verticalEncoderLeft = left odometry encoder
     * @param verticalEncoderRIght = right odometry encoder
     * @param horizontalEncoder = horizontal odometry encoder
     * @param COUNTS_PER_INCH = Number of encoder counts per inch that the robot moved (test to find constant)
     * @param threadSleepDelay = Millisecond delay between checks of this trhead (50 - 75 milliseconds recommended)
     */
    public Odometry(DcMotor verticalEncoderLeft, DcMotor verticalEncoderRIght, DcMotor horizontalEncoder, double COUNTS_PER_INCH, int threadSleepDelay)
    {
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRIght;
        this.horizontalEncoder = horizontalEncoder;
        sleepTime = threadSleepDelay;
        this.COUNTS_PER_INCH = COUNTS_PER_INCH;

        robotEncoderWheelDist = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    }

    /**
    * Updates the global coordinates (x, y, theta) using the odometry wheels
    */
    private void globalPosUdpate()
    {
//        Get current positions
        verticalLeftEncoderWheelPos = (verticalEncoderLeft.getCurrentPosition() * verticalLeftEncoderPosMultiplier);
        verticalRightEncoderWheelPos = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPosMultiplier);

        double leftChange = verticalLeftEncoderWheelPos - lastVerticalLeftEncoderWheelPos;
        double rightChange = verticalRightEncoderWheelPos - lastVerticalRightEncodetWheelPos;

//        Calculate angle
        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDist);
        robotOrientationRad += changeInRobotOrientation;

//        Split into tangent and normal components
        normalEncoderWheelPos = horizontalEncoder.getCurrentPosition()*normalEncoderPosMultiplier;
        double rawHorizontalChange = normalEncoderWheelPos - lastNormalEncoderWheelPos;
        double horizontalChange= rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);

//        T = tangent (parallel)
//        N = normal (perpendicular)
        double T = (rightChange + leftChange) / 2;
        double N = horizontalChange;

//        Calculate and update position values
        robotGlobalXPos = robotGlobalXPos + (T*Math.sin(robotOrientationRad) + N*Math.cos(robotOrientationRad));
        robotGlobalYPos = robotGlobalYPos + (T*Math.cos(robotOrientationRad) - N*Math.sin(robotOrientationRad));

//        Save current encoder positions for use in next update
        lastVerticalLeftEncoderWheelPos = verticalLeftEncoderWheelPos;
        lastVerticalRightEncodetWheelPos = verticalRightEncoderWheelPos;
        lastNormalEncoderWheelPos = normalEncoderWheelPos;
    }

    public double getXPosRaw() { return robotGlobalXPos; }

    public double getYPosRaw() { return robotGlobalYPos; }

    public double getXPos() { return robotGlobalXPos / COUNTS_PER_INCH; }

    public double getYPos() { return robotGlobalYPos / COUNTS_PER_INCH; }

    public double getOrientation() { return wrapAngle(Math.toDegrees(robotOrientationRad)); }

    public void stop() { isRunning = false; }

    private double wrapAngle(double angle) {
        if(angle < 0) return angle + 360;
        else if(angle >= 360) return angle - 360;
        else return angle;
    }
    public void reverseLeft() {
        if(verticalLeftEncoderPosMultiplier == 1)
            verticalLeftEncoderPosMultiplier = -1;
        else
            verticalLeftEncoderPosMultiplier = 1;
    }

    public void reverseRight() {
        if(verticalRightEncoderPosMultiplier == 1)
            verticalRightEncoderPosMultiplier = -1;
        else
            verticalRightEncoderPosMultiplier = 1;
    }

    public void reverseNormal() {
        if(normalEncoderPosMultiplier == 1)
            normalEncoderPosMultiplier = -1;
        else
            normalEncoderPosMultiplier = 1;
    }



    @Override
    public void run() {
        while(isRunning) {
            globalPosUdpate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
