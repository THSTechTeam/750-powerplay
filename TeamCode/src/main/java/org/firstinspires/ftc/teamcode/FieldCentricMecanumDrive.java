package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.util.DriveMode;
import org.firstinspires.ftc.teamcode.util.MecanumDriveManager;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
@TeleOp(name="Field Centric Mecanum Drive", group="TeleOp")
public class FieldCentricMecanumDrive extends LinearOpMode {
    private MecanumDriveManager drive;

    private CRServo servoGrabber = null;
    private CRServo servoPivot = null;

    /** Change these values to modify motor/servo positions and speeds ****************************/
    private PIDController liftController;
    public static double LIFT_KP = 0.0005;
    public static double LIFT_KI = 0.0001;
    public static double LIFT_KD = 0.00001;

    public static int MANUAL_LIFT_INCREMENT = 50;
    
    // These values are marked as public to allow the dashboard to display them for tuning. 
    public static int LIFT_LEVEL_0 = 100;
    public static int LIFT_LEVEL_1 = 1500;
    public static int LIFT_LEVEL_2 = 2400;
    public static int LIFT_LEVEL_3 = 3400;
    public static double LIFT_POWER = 1;

    private static final double PIVOT_POWER = 0.7;
    private static final double PIVOT_FRONT_POSITION = 1;
    private static final double PIVOT_BACK_POSITION = 0.25;

    private static final double GRABBER_POWER = 0.7;

    /**********************************************************************************************/

    public static double lowPowerFactor = 0.4;
    public static double highPowerFactor = 0.6;

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
                DcMotorSimple.Direction.FORWARD
            );
        liftController.setEncoderConstraints(0, 3700);
        liftController.setMaxMotorPower(LIFT_POWER);

        servoPivot = hardwareMap.get(CRServo.class, "servoPivot");
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

        while (opModeIsActive()) {
            telemetry.addData("Currently at", " at %7d", liftController.getCurrentPosition());
            telemetry.update();
            liftController.update();

            /** Lift Code *************************************************************************/

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

            if (gamepad2.dpad_up) {
                liftController.setTargetPosition(liftController.getTargetPosition() + MANUAL_LIFT_INCREMENT);
            } else if (gamepad2.dpad_down) {
                liftController.setTargetPosition(liftController.getTargetPosition() - MANUAL_LIFT_INCREMENT);
            }

            if (gamepad2.dpad_right) {
                liftController.resetEncoder();
            }

            /** Pivot Code ************************************************************************/

            if (Math.abs(gamepad2.right_stick_x) > 0.1){
                servoPivot.setPower(gamepad2.right_stick_x * PIVOT_POWER);
            }
            else {
                servoPivot.setPower(0);
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

            idle();
        }
    }
}
