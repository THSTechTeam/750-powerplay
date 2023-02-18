package org.firstinspires.ftc.teamcode.common.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import com.qualcomm.robotcore.util.ElapsedTime

class PIDController(
    var kP: Double,
    var kI: Double,
    var kD: Double,
    var motor: DcMotorEx,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD
) {
    var currentError = 0
    var lastError = 0

    var positionSet = false

    var timeStep = 1.0
    var lastTime = 1.0
    var elapsedTime = ElapsedTime()

    var maxMotorPower = 1.0
        set(value) {
            field = when {
                value > 1.0 -> 1.0
                value < 0.0 -> 0.0
                else -> value
            }
        }

    val currentPosition
        get() = motor.currentPosition
    val motorPower
        get() = motor.power

    var useEncoderConstraints = false
    var minEncoderConstraint = 0
    var maxEncoderConstraint = 0

    init {
        val motorConfigurationType = motor.motorType.clone()
        motorConfigurationType.achieveableMaxRPMFraction = 1.0
        motor.motorType = motorConfigurationType

        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = direction
    }

    var targetPosition = 0
        set(value) {
            field = if (useEncoderConstraints) {
                when {
                    value < minEncoderConstraint -> minEncoderConstraint
                    value > maxEncoderConstraint -> maxEncoderConstraint
                    else -> value
                }
            } else {
                value
            }

            positionSet = true

            // Start the motor with proportional control.
            // This is done to simplify initialization of the lastTime variable.
            var power = kP * (targetPosition - motor.currentPosition)
            power = when {
                power > maxMotorPower -> maxMotorPower
                power < -maxMotorPower -> -maxMotorPower
                else -> power
            }
            motor.power = power
            lastTime = elapsedTime.milliseconds()
        }

    fun update() {
        if (!positionSet) {
            return
        }

        timeStep = elapsedTime.milliseconds() - lastTime
        currentError = targetPosition - motor.currentPosition

        val p = kP * currentError
        val i = kI * currentError * timeStep
        val d = (kD * (currentError - lastError)) / timeStep
        var power = p + i + d

        power = when {
            power > maxMotorPower -> maxMotorPower
            power < -maxMotorPower -> -maxMotorPower
            else -> power
        }

        motor.power = power
        lastError = currentError
        lastTime = elapsedTime.milliseconds()
    }

    fun resetEncoder() {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun isBusy() = motor.isBusy

    fun setEncoderConstraints(minEncoderConstraint: Int, maxEncoderConstraint: Int) {
        useEncoderConstraints = true
        this.minEncoderConstraint = minEncoderConstraint
        this.maxEncoderConstraint = maxEncoderConstraint
    }

    fun setConstants(kP: Double, kI: Double, kD: Double) {
        this.kP = kP
        this.kI = kI
        this.kD = kD
    }
}
