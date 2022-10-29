package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Drive Motor Diagnostic", group="TeleOp")
public class DriveMotorDiagnostic extends LinearOpMode {
    // General gamepad controller, all robot specific methods will be placed inside
    // the robot specific implementation of GamepadController.
    private class GamepadControllerBase {
        protected Gamepad gamepad;
        protected Gamepad previous;

        protected GamepadControllerBase() {
            gamepad  = new Gamepad();
            previous = new Gamepad();
        }

        // Must be called at the beginning of each while opModeIsActive() loop.
        protected void update() {
            try {
                previous.copy(gamepad);
                gamepad.copy(gamepad1);
            } catch (RobotCoreException e) {
                // Swallow exception, gamepad1 should always be valid.
            }
        }
    }

    private class GamepadController extends GamepadControllerBase {
        public boolean isPressedX() {
            return gamepad.x;
        }

        public boolean isPressedA() {
            return gamepad.a;
        }

        public boolean isPressedB() {
            return gamepad.b;
        }

        public boolean isPressedY() {
            return gamepad.y;
        }
    }

    private static class MotorPowerFactors {
        public static final double lowDrive = 0.3;
    }

    private final GamepadController gamepadController = new GamepadController();

    @Override
    public void runOpMode() throws InterruptedException {
        gamepadController.update();
        
        // Initialize motors.
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "motorBackRight");

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            // If a pressed spin front left.
            if (gamepadController.isPressedX()) {
                frontLeft.setPower(MotorPowerFactors.lowDrive);
            } else {
                frontLeft.setPower(0);
            }

            // If b pressed spin back left.
            if (gamepadController.isPressedA()) {
                backLeft.setPower(MotorPowerFactors.lowDrive);
            } else {
                backLeft.setPower(0);
            }

            // If x pressed spin front right.
            if (gamepadController.isPressedB()) {
                frontRight.setPower(MotorPowerFactors.lowDrive);
            } else {
                frontRight.setPower(0);
            }

            // If y pressed spin back right.
            if (gamepadController.isPressedY()) {
                backRight.setPower(MotorPowerFactors.lowDrive);
            } else {
                backRight.setPower(0);
            }
        }
    }
}
