package org.firstinspires.ftc.teamcode;
    
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontanires.MotorConfigurationType;

/*
 * Motor controller for handling acceleration and deceleration using,
 * built in PID controls.
 */
public class LiftController {
    private final DcMotorEx motor;

    private double maxMotorPower;
    private double accelerationShoulder;
    private double powerIncrement;
    private double distancePowerIncrement;

    public LiftController(
            DcMotorEx motor, 
            double maxMotorPower, 
            double accelerationShoulder, 
            double powerIncrement
        ) {
        this.motor = motor;
        this.maxMotorPower = maxMotorPower;
        this.accelerationShoulder = accelerationShoulder;
        this.powerIncrement = powerIncrement;
        
        // Calculate the distance between power increments in order to get the power to its max value.
        this.distancePowerIncrement = (maxMotorPower - 0.1) / powerIncrement * accelerationShoulder;

        MotorConfigurationType motorConfigurationType = this.motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        this.motor.setMotorType(motorConfigurationType);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setMaxMotorPower(double maxMotorPower) {
        this.maxMotorPower = maxMotorPower;
    }

    public void setAccelerationShoulder(double accelerationShoulder) {
        this.accelerationShoulder = accelerationShoulder;
    }

    public void setTargetPosition(int targetPosition) {
        this.motor.setTargetPosition(targetPosition);
        elapsedTime.reset();
    }

    public void run() {
        double motorPower = 0.1; // Current / starting motor power.
        double lastPowerIncrement = 0; // Distance of the last power increment.

        while (this.motor.isBusy()) {
            if (!this.isWithinShoulder()) {
                continue;
            }

            if (lastPowerIncrement + this.distancePowerIncrement < 
                    Math.abs(this.motor.getTargetPosition() - this.motor.getCurrentPosition())) {
                motorPower += this.powerIncrement;
                lastPowerIncrement += this.distancePowerIncrement;
            }

            if (motorPower > this.maxMotorPower) {
                motorPower = this.maxMotorPower;
            }

            this.motor.setPower(motorPower);
        }
    }

    public boolean isWithinShoulder() {
        return Math.abs(this.motor.getTargetPosition() - this.motor.getCurrentPosition()) < this.accelerationShoulder;
    }
}
