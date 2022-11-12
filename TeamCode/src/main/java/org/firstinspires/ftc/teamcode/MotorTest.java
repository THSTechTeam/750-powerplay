package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.util.Range;

@TeleOp
public class MotorTest extends LinearOpMode {
    private class GamepadControllerBase {
        protected Gamepad gamepad;
        protected Gamepad previous;

        protected GamepadControllerBase() {
            gamepad  = new Gamepad();
            previous = new Gamepad();
        }

        // Must be called at the beginning of each while opModeIsActive() loop.
        // NOTE: 4097 driver station assignees controller to gamepad2 by default.
        protected void update() {
            try {
                previous.copy(gamepad);
                gamepad.copy(gamepad2);
            } catch (RobotCoreException e) {
                // Swallow exception, gamepad2 should always be valid.
            }
        }
    }

    private class GamepadController extends GamepadControllerBase {
        public boolean isPressedA() {
            return gamepad.a && !previous.a;
        }

        public boolean isPressedB() {
            return gamepad.b && !previous.b;
        }

        public boolean isPressedX() {
            return gamepad.x && !previous.x;
        }

        public boolean isPressedY() {
            return gamepad.y && !previous.y;
        }

        public boolean isPressedDPadUp() {
            return gamepad.dpad_up && !previous.dpad_up;
        }

        public boolean isPressedDPadDown() {
            return gamepad.dpad_down && !previous.dpad_down;
        }

        public boolean isPressedDPadLeft() {
            return gamepad.dpad_left && !previous.dpad_left;
        }

        public boolean isPressedDPadRight() {
            return gamepad.dpad_right && !previous.dpad_right;
        }
    }

    private final GamepadController gamepadController = new GamepadController();

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor[] mecanumMotors = {
            hardwareMap.get(DcMotor.class, "motorFrontLeft"),
            hardwareMap.get(DcMotor.class, "motorFrontRight"),
            hardwareMap.get(DcMotor.class, "motorBackLeft"),
            hardwareMap.get(DcMotor.class, "motorBackRight")
        };

        waitForStart();

        while (opModeIsActive()) {
            gamepadController.update();

            if (gamepadController.isPressedA()) {
                mecanumMotors[0].setPower(1);
            } else if (gamepadController.isPressedB()) {
                mecanumMotors[0].setPower(0);
            }

            if (gamepadController.isPressedX()) {
                mecanumMotors[1].setPower(1);
            } else if (gamepadController.isPressedY()) {
                mecanumMotors[1].setPower(0);
            }

            if (gamepadController.isPressedDPadUp()) {
                mecanumMotors[2].setPower(1);
            } else if (gamepadController.isPressedDPadDown()) {
                mecanumMotors[2].setPower(0);
            }

            if (gamepadController.isPressedDPadLeft()) {
                mecanumMotors[3].setPower(1);
            } else if (gamepadController.isPressedDPadRight()) {
                mecanumMotors[3].setPower(0);
            }
        }
    }
}
