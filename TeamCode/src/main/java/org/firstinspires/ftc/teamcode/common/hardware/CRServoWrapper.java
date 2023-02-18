package org.firstinspires.ftc.teamcode.common.hardware;

import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Basic java wrapper for the CRServo.
 * 
 * For some reason in kotlin the servo.Direction enum is not accessible, so this is a simple workaround.
 */
public class CRServoWrapper {
    public CRServo servo;

    public enum Direction {
        FORWARD, REVERSE
    }

    public CRServoWrapper(CRServo servo) {
        this.servo = servo;
    }

    public void setDirection(Direction direction) {
        if (direction == Direction.FORWARD) {
            servo.setDirection(CRServo.Direction.FORWARD);
        } else {
            servo.setDirection(CRServo.Direction.REVERSE);
        }
    }

    public void setPower(double power) {
        servo.setPower(power);
    }
}
