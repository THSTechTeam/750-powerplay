// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.robotcore.hardware.Servo;

// @TeleOp(name="Mecanum Drive + Arm Controls", group="TeleOp")
// public class PrimaryDrive extends LinearOpMode {
//     private DcMotor motorFrontLeft = null;
//     private DcMotor motorFrontRight = null;
//     private DcMotor motorBackLeft = null;
//     private DcMotor motorBackRight = null;
//     private DcMotor motorLift = null;
//     private Servo grabberLeft = null;
//     private Servo grabberRight = null;
//     private Servo servoPivot = null;

//     /** Change these values to modify motor/servo positions and speeds ****************************/

//     private static final int LIFT_BOTTOM_POSITION = 0;
//     private static final int LIFT_TOP_POSITION = 5000;
//     private static final double LIFT_SPEED = 0.5;

//     private static final double PIVOT_FRONT_POSITION = 0;
//     private static final double PIVOT_BACK_POSITION = 1;

//     private static final double GRABBER_OPEN_POSITION = 0;
//     private static final double GRABBER_CLOSED_POSITION = 1;

//     /**********************************************************************************************/

//     private final double lowPowerFactor = 0.3;
//     private final double highPowerFactor = 0.75;

//     private double motorPowerFactor = lowPowerFactor;

//     private double getPowerFactor(final double previousPowerFactor) {
//        if (gamepad1.a) {
//            return lowPowerFactor;
//        } else if (gamepad1.b) {
//            return highPowerFactor;
//        } else {
//            return previousPowerFactor;
//        }
//    }


//     @Override
//     public void runOpMode() throws InterruptedException {
//        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
//        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
//        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
//        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
//        motorLift = hardwareMap.dcMotor.get("motorLift");
//        servoPivot = hardwareMap.servo.get("servoPivot");
//        grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
//        grabberRight = hardwareMap.get(Servo.class, "grabberRight");

//         grabberLeft.setDirection(Servo.Direction.FORWARD);
//         grabberRight.setDirection(Servo.Direction.FORWARD);

//         // reverse left side motors
//        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

//         // TODO: Brandon Note - IMU Code broken on vertical hubs
//         // retrieve imu from hardware map
// //        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
// //        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
// //        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
// //        imu.initialize(parameters);

//         waitForStart();

//         if (isStopRequested()) {
//             return;
//         }

//         while (opModeIsActive()) {
//                 telemetry.addData("Currently at", " at %7d", motorLift.getCurrentPosition());
//                 telemetry.update();

//             /** Lift Code *************************************************************************/

//                 // Lift to top
//                 if (gamepad1.y && !gamepad1.x) {
//                     motorLift.setTargetPower(LIFT_TOP_POSITION);
//                     motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                     motorLift.setPower(LIFT_SPEED);
//                 }
//                 // Return lift to bottom
//                 else if (gamepad1.x && !gamepad1.y) {
//                     motorLift.setTargetPower(LIFT_BOTTOM_POSITION);
//                     motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                     motorLift.setPower(LIFT_SPEED);
//                 }
//                 // If neither are pressed, just stop for now
//                 else {
//                     motorLift.setPower(0);
//                 }

//             /** Pivot Code ************************************************************************/

//                 // Pivot to front
//                 if (gamepad1.dpad_left && !gamepad1.dpad_right){
//                     servoPivot.setPosition(PIVOT_FRONT_POSITION);
//                 }
//                 // Pivot to back
//                 else if (gamepad1.dpad_right && !gamepad1.dpad_left){
//                     servoPivot.setPosition(PIVOT_BACK_POSITION);
//                 }

//            /** Grabber Code ***********************************************************************/

//                 telemetry.addData("grabberLeft Position", "%.2f", grabberLeft.getPower());
//                 telemetry.addData("grabberRight Position", "%.2f", grabberRight.getPower());
//                 telemetry.update();

//                 // Open grabber
//                 if (gamepad1.left_bumper && !gamepad1.right_bumper) {
//                     grabberLeft.setPosition(GRABBER_OPEN_POSITION);
//                     grabberRight.setPosition(GRABBER_OPEN_POSITION);
//                 }
//                 // Close grabber
//                 else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
//                     grabberLeft.setPosition(GRABBER_CLOSED_POSITION);
//                     grabberRight.setPosition(GRABBER_CLOSED_POSITION);
//                 }

//             /** Drive Code ************************************************************************/

//             // 4097 driver station assignees controller to gamepad2 by default
//             final double y = -gamepad1.left_stick_y; // reversed
//             final double x = -(gamepad1.left_stick_x * 1.0); // imperfect strafing fix & reversed
//             final double rx = gamepad1.right_stick_x;

//             // TODO: Brandon Note - IMU Code broken on vertical hubs
//             // final double botHeading = imu.getAngularOrientation().firstAngle;

//             // x / y offsets
//             // final double rotY = y * Math.cos(botHeading) + x * Math.sin(botHeading);
//             // final double rotX = -y * Math.sin(botHeading) + x * Math.cos(botHeading);
//             final double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

//             double[] motorPowers = {
//                     (y + x + rx) / denominator, // front left
//                     (y - x + rx) / denominator, // back left
//                     (y - x - rx) / denominator, // front right
//                     (y + x - rx) / denominator, // back right
//             };

//             motorPowerFactor = getPowerFactor(motorPowerFactor);

//             for (int i = 0; i < motorPowers.length; i++) {
//                 motorPowers[i] *= motorPowerFactor;
//             }

//             motorFrontLeft.setPower(motorPowers[0]);
//             motorBackLeft.setPower(motorPowers[1]);
//             motorFrontRight.setPower(motorPowers[2]);
//             motorBackRight.setPower(motorPowers[3]);

//             idle();
//         }

//         motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     }
// }

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Mecanum Drive + Arm Controls", group="TeleOp")
public class PrimaryDrive extends LinearOpMode {
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
                gamepad.copy(gamepad1);
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

        public boolean isPressedDPadLeft() {
            return gamepad.dpad_left && !previous.dpad_left;
        }

        public boolean isPressedDPadRight() {
            return gamepad.dpad_right && !previous.dpad_right;
        }

        public boolean isPressedLeftBumper() {
            return gamepad.left_bumper && !previous.left_bumper;
        }

        public boolean isPressedRightBumper() {
            return gamepad.right_bumper && !previous.right_bumper;
        }
    }

    private class GamepadController2 extends GamepadController {
        @Override
        public void update() {
            try {
                previous.copy(gamepad);
                gamepad.copy(gamepad2);
            } catch (RobotCoreException e) {
                // Swallow exception, gamepad1 should always be valid.
            }
        }

        @Override
        public boolean isPressedA() {
            return gamepad.a;
        }

        @Override
        public boolean isPressedY() {
            return gamepad.y;
        }

        @Override
        public boolean isPressedDPadRight() {
            return gamepad.dpad_right;
        }

        @Override
        public boolean isPressedDPadLeft() {
            return gamepad.dpad_left;
        }
    }

    private static class MotorPowerFactors {
        public static final double lowDrive  = 0.3;
        public static final double highDrive = 0.6;
    }

    private double getDrivePowerFactor(final double previousPowerFactor) {
        if (!gamepadController.isPressedA()) {
            return previousPowerFactor;
        }

        if (previousPowerFactor == MotorPowerFactors.lowDrive) {
            return MotorPowerFactors.highDrive;
        } else {
            return MotorPowerFactors.lowDrive;
        }
    }

    private static class ServoPositions {
        // TODO: Left grabber servo seems broken.
        public static final double grabberClosed = 0.0;
        public static final double grabberOpen   = 1.0;

        // TODO: Pivot servo seems broken. More accurately the default angle needs to be reset.
        public static final double pivotFront = 0.0;
        public static final double pivotBack  = 1.0;
    }

    private final GamepadController gamepadController  = new GamepadController();
    private final GamepadController2 gamepadController2 = new GamepadController2();

    @Override
    public void runOpMode() throws InterruptedException {
        double driveMotorPowerFactor = MotorPowerFactors.lowDrive;

        DcMotor[] mecanumMotors = {
            hardwareMap.get(DcMotor.class, "motorFrontLeft"),
            hardwareMap.get(DcMotor.class, "motorBackLeft"),
            hardwareMap.get(DcMotor.class, "motorFrontRight"),
            hardwareMap.get(DcMotor.class, "motorBackRight"),
        };

        // Reverse left side motors.
        mecanumMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor motorLift         = hardwareMap.get(DcMotor.class, "motorLift");
        CRServo servoGrabberLeft  = hardwareMap.get(CRServo.class, "servoGrabberLeft");
        CRServo servoGrabberRight = hardwareMap.get(CRServo.class, "servoGrabberRight");
        CRServo servoPivot        = hardwareMap.get(CRServo.class, "servoPivot");

        // Set breaking on lift motor.
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            gamepadController.update();
            gamepadController2.update();

            // Drive code.
            final double ly = -gamepadController.gamepad.left_stick_y; // reversed
            final double lx = -gamepadController.gamepad.left_stick_x; // reversed
            final double rx = gamepadController.gamepad.right_stick_x;
            final double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

            final double[] motorPowers = {
                (ly + lx + rx) / denominator, // front left
                (ly - lx + rx) / denominator, // back left
                (ly - lx - rx) / denominator, // front right
                (ly + lx - rx) / denominator, // back right
            };

            driveMotorPowerFactor = getDrivePowerFactor(driveMotorPowerFactor);

            for (int i = 0; i < motorPowers.length; i++) {
                mecanumMotors[i].setPower(motorPowers[i] * driveMotorPowerFactor);
            }

            // Grabber code.
            telemetry.addData("Grabber Left Position", servoGrabberLeft.getPower());
            telemetry.addData("Grabber Right Position", servoGrabberRight.getPower());

            if (gamepadController2.gamepad.left_stick_x > 0 || gamepadController2.gamepad.left_stick_x < 0) {
                servoGrabberLeft.setPower(gamepadController2.gamepad.left_stick_x);
            } else {
                servoGrabberLeft.setPower(0.0);
            }

            if (gamepadController2.gamepad.right_stick_x > 0 || gamepadController2.gamepad.right_stick_x < 0) {
                servoGrabberRight.setPower(gamepadController2.gamepad.right_stick_x);
            } else {
                servoGrabberRight.setPower(0.0);
            }

            // Pivot code.
            telemetry.addData("Pivot Position", servoPivot.getPower());

            if (gamepadController2.isPressedDPadRight()) {
                servoPivot.setPower(0.5);
            } else {
                servoPivot.setPower(0.0);
            }

            if (gamepadController2.isPressedDPadLeft()) {
                servoPivot.setPower(-0.5);
            } else {
                servoPivot.setPower(0.0);
            }

            // Lift code.
            if (gamepadController2.isPressedY()) {
                motorLift.setPower(0.2);
            } else if (gamepadController2.isPressedA()) {
                motorLift.setPower(-0.05);
            } else {
                motorLift.setPower(0.0);
            }

            telemetry.update();
            idle();
        }
    }
}
