package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.String;

@TeleOp(name="Servo / Motor Test", group="TeleOp")
public class ServoTest extends LinearOpMode {
    private class GamepadControllerBase {
        protected Gamepad gamepad;
        protected Gamepad previous;

        protected GamepadControllerBase() {
            this.gamepad  = new Gamepad();
            this.previous = new Gamepad();
        }

        protected void update(Gamepad gamepad) {
            try {
                this.previous.copy(this.gamepad);
                this.gamepad.copy(gamepad);
            } catch (RobotCoreException e) {
                // Swallow exception, controller is just unplugged at the moment.
            }
        }
    }

    public class GamepadController extends GamepadControllerBase {
        public boolean isPressedA() {
            return this.gamepad.a && !this.previous.a;
        }

        public boolean isPressedB() {
            return this.gamepad.b && !this.previous.b;
        }

        public boolean isPressedX() {
            return this.gamepad.x && !this.previous.x;
        }

        public boolean isPressedY() {
            return this.gamepad.y && !this.previous.y;
        }

        public boolean isPressedDPadUp() {
            return this.gamepad.dpad_up && !this.previous.dpad_up;
        }

        public boolean isPressedDPadDown() {
            return this.gamepad.dpad_down && !this.previous.dpad_down;
        }

        public boolean isPressedDPadLeft() {
            return this.gamepad.dpad_left && !this.previous.dpad_left;
        }

        public boolean isPressedDPadRight() {
            return this.gamepad.dpad_right && !this.previous.dpad_right;
        }

        public boolean isPressedLeftBumper() {
            return this.gamepad.left_bumper && !this.previous.left_bumper;
        }

        public boolean isPressedRightBumper() {
            return this.gamepad.right_bumper && !this.previous.right_bumper;
        }

        public boolean isPressedLeftStickButton() {
            return this.gamepad.left_stick_button && !this.previous.left_stick_button;
        }

        public boolean isPressedRightStickButton() {
            return this.gamepad.right_stick_button && !this.previous.right_stick_button;
        }

        public boolean isPressedStart() {
            return this.gamepad.start && !this.previous.start;
        }

        public boolean isPressedBack() {
            return this.gamepad.back && !this.previous.back;
        }

        public boolean isPressedGuide() {
            return this.gamepad.guide && !this.previous.guide;
        }

        public boolean isPressedLeftTrigger() {
            return this.gamepad.left_trigger > 0.5 && this.previous.left_trigger <= 0.5;
        }

        public boolean isPressedRightTrigger() {
            return this.gamepad.right_trigger > 0.5 && this.previous.right_trigger <= 0.5;
        }

        public boolean isHeldA() {
            return this.gamepad.a && this.previous.a;
        }

        public boolean isHeldB() {
            return this.gamepad.b && this.previous.b;
        }

        public boolean isHeldX() {
            return this.gamepad.x && this.previous.x;
        }

        public boolean isHeldY() {
            return this.gamepad.y && this.previous.y;
        }

        public boolean isHeldDPadUp() {
            return this.gamepad.dpad_up && this.previous.dpad_up;
        }

        public boolean isHeldDPadDown() {
            return this.gamepad.dpad_down && this.previous.dpad_down;
        }

        public boolean isHeldDPadLeft() {
            return this.gamepad.dpad_left && this.previous.dpad_left;
        }

        public boolean isHeldDPadRight() {
            return this.gamepad.dpad_right && this.previous.dpad_right;
        }

        public boolean isHeldLeftBumper() {
            return this.gamepad.left_bumper && this.previous.left_bumper;
        }

        public boolean isHeldRightBumper() {
            return this.gamepad.right_bumper && this.previous.right_bumper;
        }

        public boolean isHeldLeftStickButton() {
            return this.gamepad.left_stick_button && this.previous.left_stick_button;
        }

        public boolean isHeldRightStickButton() {
            return this.gamepad.right_stick_button && this.previous.right_stick_button;
        }

        public boolean isHeldStart() {
            return this.gamepad.start && this.previous.start;
        }

        public boolean isHeldBack() {
            return this.gamepad.back && this.previous.back;
        }

        public boolean isHeldGuide() {
            return this.gamepad.guide && this.previous.guide;
        }

        public boolean isHeldLeftTrigger() {
            return this.gamepad.left_trigger > 0.5 && this.previous.left_trigger > 0.5;
        }

        public boolean isHeldRightTrigger() {
            return this.gamepad.right_trigger > 0.5 && this.previous.right_trigger > 0.5;
        }
    }

    private class ServoController {
        private Servo servo;
        public double startPosition;
        public double endPosition;

        public ServoController(Servo servo, double startPosition, double endPosition) {
            this.servo = servo;
            this.startPosition = startPosition;
            this.endPosition = endPosition;
        }

        public void changeStartPosition(double delta) {
            this.startPosition += delta;
        }

        public void changeEndPosition(double delta) {
            this.endPosition += delta;
        }

        public void setPosition(double position) {
            this.servo.setPosition(position);
        }

        public double getPosition() {
            return this.servo.getPosition();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadController gamepad1Controller = new GamepadController();
        GamepadController gamepad2Controller = new GamepadController();

        ServoController servoGrabberLeft  = new ServoController(hardwareMap.servo.get("servoGrabberLeft"), 0.4, 0.6);
        ServoController servoGrabberRight = new ServoController(hardwareMap.servo.get("servoGrabberRight"), 0.4, 0.6);

        waitForStart();

        while (opModeIsActive()) {
            gamepad1Controller.update(gamepad1);
            gamepad2Controller.update(gamepad2);

            String currentMode = null;

            // Update the current mode between moving and setting the servo positions.
            if (gamepad1Controller.isPressedDPadUp()) {
                currentMode = "move";
            } else if (gamepad1Controller.isPressedDPadDown()) {
                currentMode = "set";
            } else if (currentMode == null) {
                currentMode = "move";
            }

            telemetry.addData("Current Mode", currentMode);

            if (currentMode == "set") {
                // Select the servo to change the start / end position of.
                if (gamepad1Controller.isPressedDPadLeft()) {
                    servoGrabberLeft.changeStartPosition(0.05);
                } else if (gamepad1Controller.isPressedDPadRight()) {
                    servoGrabberLeft.changeStartPosition(-0.05);
                } 
                
                if (gamepad2Controller.isPressedDPadLeft()) {
                    servoGrabberRight.changeStartPosition(0.05);
                } else if (gamepad2Controller.isPressedDPadRight()) {
                    servoGrabberRight.changeStartPosition(-0.05);
                }

                telemetry.addData("Servo Grabber Left Start Position: ", servoGrabberLeft.startPosition);
                telemetry.addData("Servo Grabber Left End Position: ", servoGrabberLeft.endPosition);
                telemetry.addData("Servo Grabber Right Start Position: ", servoGrabberRight.startPosition);
                telemetry.addData("Servo Grabber Right End Position: ", servoGrabberRight.endPosition);
            } else if (currentMode == "move") {
                // Move the servo to the start / end position.
                if (gamepad1Controller.isPressedDPadLeft()) {
                    servoGrabberLeft.setPosition(servoGrabberLeft.startPosition);
                } else if (gamepad1Controller.isPressedDPadRight()) {
                    servoGrabberLeft.setPosition(servoGrabberLeft.endPosition);
                } 

                if (gamepad2Controller.isPressedDPadLeft()) {
                    servoGrabberRight.setPosition(servoGrabberRight.startPosition);
                } else if (gamepad2Controller.isPressedDPadRight()) {
                    servoGrabberRight.setPosition(servoGrabberRight.endPosition);
                }

                telemetry.addData("Servo Grabber Left Position: ", servoGrabberLeft.getPosition());
                telemetry.addData("Servo Grabber Right Position: ", servoGrabberRight.getPosition());
            }

            telemetry.update();
            idle();
        }
    }
}
