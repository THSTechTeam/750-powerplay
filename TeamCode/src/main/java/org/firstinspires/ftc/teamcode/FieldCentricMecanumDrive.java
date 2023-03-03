package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.util.DriveMode;
import org.firstinspires.ftc.teamcode.util.MecanumDriveManager;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
@TeleOp(name="Field Centric Mecanum Drive", group="TeleOp")
public class FieldCentricMecanumDrive extends LinearOpMode {
    private MecanumDriveManager drive;

    private CRServo servoGrabber = null;
    private Servo servoPivot = null;

    /** Change these values to modify motor/servo positions and speeds ****************************/
    private PIDController liftController;
    public static double LIFT_KP = 0.003;
    public static double LIFT_KI = 0.0;
    public static double LIFT_KD = 0.001;

    public static int MANUAL_LIFT_INCREMENT = 60;
    
    // These values are marked as public to allow the dashboard to display them for tuning. 
    public static int LIFT_LEVEL_0 = 100;
    public static int LIFT_LEVEL_1 = 1900;
    public static int LIFT_LEVEL_2 = 2980;
    public static int LIFT_LEVEL_3 = 4200;
    public static double LIFT_POWER = 0.9;

    public static int START_STACK_POSITION = 700;
    public static int STACK_POSITION_CONE_OFFSET = 150;

    public static double PIVOT_POWER = 0.03;
    public static double PIVOT_FRONT_POSITION = 0.0;
    public static double PIVOT_SIDE_POSITION = 0.5;
    public static double PIVOT_BACK_POSITION = 1.0;

    private static final double GRABBER_POWER = 0.7;

    // private VoltageSensor batteryVoltageSensor;

    /**********************************************************************************************/

    public static double lowPowerFactor = 0.5;
    public static double highPowerFactor = 0.7;

    private int currentLiftLevel = 0;
    private double motorPowerFactor = lowPowerFactor;

    private double getPowerFactor(double previousPowerFactor) {
       if (gamepad1.a) {
           return lowPowerFactor;
       } else if (gamepad1.b) {
           return highPowerFactor;
       } else {
           return previousPowerFactor;
       }
   }

    @Override
    public void runOpMode() throws InterruptedException {
        liftController = new PIDController(
                LIFT_KP, 
                LIFT_KI, 
                LIFT_KD,
                hardwareMap.get(DcMotorEx.class, "motorLift"),
                DcMotorSimple.Direction.REVERSE
            );
        liftController.setMaxMotorPower(LIFT_POWER);

        // batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        servoPivot = hardwareMap.get(Servo.class, "servoPivot");
        servoGrabber = hardwareMap.get(CRServo.class, "servoGrabber");

        servoGrabber.setDirection(CRServo.Direction.REVERSE);

        // Brandon drive code init.
        drive = new MecanumDriveManager(hardwareMap);
        drive.flipY();
        drive.setMode(DriveMode.FIELD_CENTRIC);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        servoPivot.setPosition(0.5);

        while (opModeIsActive()) {
            telemetry.addData("Current Drive Power", motorPowerFactor);
            telemetry.addData("Currently at", " at %7d", liftController.getCurrentPosition());
            telemetry.update();
            liftController.update();

            /** Lift Code *************************************************************************/

            // Preset lift positions.
            if (gamepad2.x) {
                liftController.setTargetPosition(LIFT_LEVEL_1);
            }
            else if (gamepad2.y) {
                liftController.setTargetPosition(LIFT_LEVEL_2);
            }
            else if (gamepad2.b) {
                liftController.setTargetPosition(LIFT_LEVEL_3);
            }
            else if (gamepad2.a) {
                liftController.setTargetPosition(LIFT_LEVEL_0);
            } 
            else if (gamepad2.dpad_up) {
                liftController.setTargetPosition(START_STACK_POSITION);
            }
            else if (gamepad2.dpad_left) {
                liftController.setTargetPosition(START_STACK_POSITION - STACK_POSITION_CONE_OFFSET);
            }
            else if (gamepad2.dpad_down) {
                liftController.setTargetPosition(START_STACK_POSITION - (STACK_POSITION_CONE_OFFSET * 2));
            }

            // Manual lift control.
            liftController.setTargetPosition(liftController.getTargetPosition() - (gamepad2.left_stick_y * MANUAL_LIFT_INCREMENT));

            if (gamepad2.dpad_right) {
                liftController.resetEncoder();
            }

            /** Pivot Code ************************************************************************/

            if (Math.abs(gamepad2.right_stick_x) > 0.1) {
                servoPivot.setPosition(servoPivot.getPosition() - (PIVOT_POWER * gamepad2.right_stick_x));
            } else if (gamepad2.left_bumper) {
                servoPivot.setPosition(PIVOT_SIDE_POSITION);
            } else if (gamepad2.right_bumper) {
                servoPivot.setPosition(PIVOT_FRONT_POSITION);
            } else if (gamepad2.left_trigger > 0.5) {
                servoPivot.setPosition(PIVOT_BACK_POSITION);
            }

           /** Grabber Code ***********************************************************************/

            // Close grabber
            if(gamepad2.right_trigger > 0.5) {
                servoGrabber.setPower(0);
            // Open grabber
            } else {
                servoGrabber.setPower(GRABBER_POWER);
            }

            /** Drive Code ************************************************************************/

            if (gamepad1.x) {
                drive.setMode(DriveMode.BOT_CENTRIC);
            } else if (gamepad1.y) {
                drive.setMode(DriveMode.FIELD_CENTRIC);
            }

            motorPowerFactor = getPowerFactor(motorPowerFactor);

            drive.setWeightedDrivePower(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    motorPowerFactor
            );

            if (gamepad1.left_trigger > 0.5) {
                drive.resetIMU();
            }

            idle();
        }
    }
}
