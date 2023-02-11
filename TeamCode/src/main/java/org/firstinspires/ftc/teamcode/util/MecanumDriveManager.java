package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Simple class to manage a four wheel mecanum drive train.
 *
 */
public class MecanumDriveManager {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private List<DcMotorEx> motors;

    private IMU imu;

    // The following is used to correct for the fact that not every drive train is the same.
    // Some will need to reverse different motors and some will need to reverse the input of different gamepad axes.
    private boolean FLIP_X        = false;
    private boolean FLIP_Y        = false;
    private boolean FLIP_ROTATION = false;

    private DriveMode mode;

    public MecanumDriveManager(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        motors = Arrays.asList(frontLeft, frontRight, backLeft, backRight);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // This section can add 2-3 seconds during the init phase of the robot
        // and can be quite annoying if you are not using it.
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT, 
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);

        setMode(DriveMode.BOT_CENTRIC);
    }

    private List<Double> getInvertedAxes(double x, double y, double turn) {
        return Arrays.asList(
                FLIP_X ? -x : x,
                FLIP_Y ? -y : y,
                FLIP_ROTATION ? -turn : turn
            );
    }

    // Used to reset the drift of the IMU.
    // WARNING: Doing this un-intentionally can cause the robot to drive in a direction that is not expected.
    // Before using this method ensure that the robot is facing the correct direction.
    public void resetIMUZeroHeading() {
        // TODO: Need to fix this. Now broken in the most recent update to the IMU.
    }

    public void setMotorPowers(double power) {
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    public void setMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void setDrivePower(double x, double y, double turn) {
        List<Double> invertedAxes = getInvertedAxes(x, y, turn);
        x    = invertedAxes.get(0);
        y    = invertedAxes.get(1);
        turn = invertedAxes.get(2);

        if (mode == DriveMode.FIELD_CENTRIC) {
            double botHeading = getRawExternalHeading();

            double adjustedX = -y * Math.sin(-botHeading) + x * Math.cos(-botHeading);
            double adjustedY =  y * Math.cos(-botHeading) + x * Math.sin(-botHeading);

            x = adjustedX;
            y = adjustedY;
        }

        double frontLeft  = y + x + turn;
        double frontRight = y - x - turn;
        double backLeft   = y - x + turn;
        double backRight  = y + x - turn;

        // Used to keep all the motor powers within the same ratio.
        double denominator = Math.max(
                Math.max(Math.abs(y), Math.abs(x)) + Math.abs(turn), 1
            );

        setMotorPowers(
                frontLeft  / denominator,
                frontRight / denominator,
                backLeft   / denominator,
                backRight  / denominator
            );
    }

    public void setWeightedDrivePower(double x, double y, double turn, double powerFactor) {
        List<Double> invertedAxes = getInvertedAxes(x, y, turn);
        x    = invertedAxes.get(0);
        y    = invertedAxes.get(1);
        turn = invertedAxes.get(2);

        if (mode == DriveMode.FIELD_CENTRIC) {
            double botHeading = getRawExternalHeading();

            double adjustedX = -y * Math.sin(-botHeading) + x * Math.cos(-botHeading);
            double adjustedY =  y * Math.cos(-botHeading) + x * Math.sin(-botHeading);

            x = adjustedX;
            y = adjustedY;
        }

        double frontLeft  = y + x + turn;
        double frontRight = y - x - turn;
        double backLeft   = y - x + turn;
        double backRight  = y + x - turn;

        // Used to keep all the motor powers within the same ratio.
        double denominator = Math.max(
                Math.max(Math.abs(y), Math.abs(x)) + Math.abs(turn), 1
            );

        setMotorPowers(
                (frontLeft  / denominator) * powerFactor,
                (frontRight / denominator) * powerFactor,
                (backLeft   / denominator) * powerFactor,
                (backRight  / denominator) * powerFactor
            );
    }

    public void flipX() { // Used for correcting variations in different drive trains.
        FLIP_X = !FLIP_X;
    }

    public void flipY() { // Used for correcting variations in different drive trains.
        FLIP_Y = !FLIP_Y;
    }

    public void flipRotation() { // Used for correcting variations in different drive trains.
        FLIP_ROTATION = !FLIP_ROTATION;
    }

    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setMode(DriveMode mode) {
        this.mode = mode;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }
}
