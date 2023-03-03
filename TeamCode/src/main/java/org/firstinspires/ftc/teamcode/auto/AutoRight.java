package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
@Disabled
@Autonomous(name="Right + Park", group="Autonomous")
public class AutoRight extends LinearOpMode {
    private static final double INSTANT_TIME = 1.0e-6;

    private ParkingLocationAnalyzer parkingLocationAnalyzer;
    private ParkingLocation parkingLocation;
    private SampleMecanumDrive drive;

    public static int START_STACK_POSITION = 0;
    public static int STACK_POSITION_CONE_OFFSET = 0;
    public static int STACK_CYCLES = 1;

    private PIDController liftController;
    public static double LIFT_KP = 0.003;
    public static double LIFT_KI = 0.0;
    public static double LIFT_KD = 0.001;

    public static int LIFT_LEVEL_0 = 150;
    public static int LIFT_LEVEL_1 = 1900;
    public static int LIFT_LEVEL_2 = 2980;
    public static int LIFT_LEVEL_3 = 4200;
    public static double LIFT_POWER = 0.7;

    private CRServo servoGrabber;

    private static double GRABBER_OPEN_POWER = 0.7;

    public static double x = 0;
    public static double y = 15;
    public static double heading = 0;
    public static double approach = 0;

    private TrajectorySequence scoreConeLowPole;
    private Trajectory resetAfterConeLowPole;
    private TrajectorySequence setupForScoringCycle;
    private TrajectorySequence grabCone;
    private Trajectory backUpToPole;
    private TrajectorySequence scoreConeHighPole;
    private Trajectory leftPark;
    private Trajectory rightPark;

    private enum State {
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        TRAJECTORY_4,
        TRAJECTORY_5,
        TRAJECTORY_6,
        PARK,
        IDLE,
    }

    private State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        liftController = new PIDController(
            LIFT_KP, 
            LIFT_KI, 
            LIFT_KD,
            hardwareMap.get(DcMotorEx.class, "motorLift"),
            DcMotorSimple.Direction.REVERSE
        );
        liftController.setMaxMotorPower(LIFT_POWER);
        liftController.resetEncoder();

        servoGrabber = hardwareMap.get(CRServo.class, "servoGrabber");
        servoGrabber.setDirection(CRServo.Direction.REVERSE);

        parkingLocationAnalyzer = new ParkingLocationAnalyzer(hardwareMap);

        scoreConeLowPole = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(1.5)
                .splineToConstantHeading(new Vector2d(3, 0), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(INSTANT_TIME, () -> {
                    liftController.setTargetPosition(LIFT_LEVEL_1);
                })
                .waitSeconds(1.5)
                .splineToSplineHeading(new Pose2d(8, -7, Math.toRadians(-30)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(INSTANT_TIME, () -> {
                    servoGrabber.setPower(GRABBER_OPEN_POWER);
                })
                .build();

        resetAfterConeLowPole = drive.trajectoryBuilder(scoreConeLowPole.end(), true)
                .splineToSplineHeading(new Pose2d(1, -4, Math.toRadians(0)), Math.toRadians(0))
                .build();

        setupForScoringCycle = drive.trajectorySequenceBuilder(resetAfterConeLowPole.end())
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(55, resetAfterConeLowPole.end().getY() + 2), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50, resetAfterConeLowPole.end().getY() + 2), Math.toRadians(0))
                .turn(Math.toRadians(90))
                .build();

        grabCone = drive.trajectorySequenceBuilder(setupForScoringCycle.end())
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(setupForScoringCycle.end().getX() + x, setupForScoringCycle.end().getY() + y), Math.toRadians(90))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(INSTANT_TIME, () -> {
                    servoGrabber.setPower(0);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(INSTANT_TIME, () -> {
                    liftController.setTargetPosition(LIFT_LEVEL_3);
                })
                .waitSeconds(1)
                .build();

        backUpToPole = drive.trajectoryBuilder(grabCone.end(), true)
                .splineToSplineHeading(new Pose2d(grabCone.end().getX(), grabCone.end().getY() - y, Math.toRadians(heading)), Math.toRadians(90))
                .build();

        scoreConeHighPole = drive.trajectorySequenceBuilder(backUpToPole.end())
                .splineToSplineHeading(new Pose2d(backUpToPole.end().getX() + x, backUpToPole.end().getY() + y, Math.toRadians(heading)), Math.toRadians(approach))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(INSTANT_TIME, () -> {
                    servoGrabber.setPower(GRABBER_OPEN_POWER);
                })
                .waitSeconds(0.5)
                .splineToSplineHeading(setupForScoringCycle.end(), Math.toRadians(approach))
                .build();

        leftPark = drive.trajectoryBuilder(grabCone.end())
                .splineToSplineHeading(new Pose2d(grabCone.end().getX(), grabCone.end().getY() + 23, Math.toRadians(0)), Math.toRadians(90))
                .build();

        rightPark = drive.trajectoryBuilder(grabCone.end(), true)
                .splineToSplineHeading(new Pose2d(grabCone.end().getX(), grabCone.end().getY() - 23, Math.toRadians(0)), Math.toRadians(90))
                .build();

        // The following loop replaces `waitForStart()`.
        while (!isStarted() && !isStopRequested()) {
            ParkingLocation newParkingLocation = parkingLocationAnalyzer.getParkingLocation();

            if (newParkingLocation != null) {
                parkingLocation = newParkingLocation;
            }

            telemetry.addData("Parking Location:", parkingLocation);
            telemetry.update();
        }

        liftController.setTargetPosition(LIFT_LEVEL_0);
        servoGrabber.setPower(0);

        currentState = State.TRAJECTORY_1;
        drive.followTrajectorySequenceAsync(scoreConeLowPole);

        int stackPosition = START_STACK_POSITION;
        int currentCycle = 0;

        // for (int i = 0; i < STACK_CYCLES; i++) {
        //     motorLift.setTargetPosition(stackPosition);
        //     drive.followTrajectorySequence(grabCone);

        //     stackPosition -= STACK_POSITION_CONE_OFFSET;
        // }

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case TRAJECTORY_1:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(resetAfterConeLowPole);
                    }
                    break;
                case TRAJECTORY_2:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_3;
                        liftController.setTargetPosition(0);
                        drive.followTrajectorySequenceAsync(setupForScoringCycle);
                    }
                case TRAJECTORY_3:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_4;
                        liftController.setTargetPosition(stackPosition);
                        drive.followTrajectorySequenceAsync(grabCone);
                    }
                    break;
                case TRAJECTORY_4:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_5;
                        // drive.followTrajectoryAsync(backUpToPole);
                    }
                    break;
                case TRAJECTORY_5:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_6;
                        // drive.followTrajectorySequenceAsync(scoreConeHighPole);
                    }
                    break;
                case TRAJECTORY_6:
                    if (!drive.isBusy()) {
                        if (currentCycle < STACK_CYCLES) {
                            currentState = State.TRAJECTORY_3;
                            stackPosition -= STACK_POSITION_CONE_OFFSET;
                            currentCycle++;
                            liftController.setTargetPosition(stackPosition);
                            // drive.followTrajectorySequenceAsync(grabCone);
                        } else {
                            currentState = State.PARK;
                            liftController.setTargetPosition(0);
                            // if (parkingLocation == ParkingLocation.LEFT) {
                            //     drive.followTrajectoryAsync(leftPark);
                            // } else {
                            //     drive.followTrajectoryAsync(rightPark);
                            // }
                        }
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        servoGrabber.setPower(GRABBER_OPEN_POWER);
                        currentState = State.IDLE;
                    }
                case IDLE:
                    break;
            }

            drive.update();
            liftController.update();
        }
    }
}
