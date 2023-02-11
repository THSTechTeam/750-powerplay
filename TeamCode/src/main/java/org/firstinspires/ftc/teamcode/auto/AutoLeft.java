package org.firstinspires.ftc.teamcode.auto;

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
@Autonomous(name="Left + Park", group="Autonomous")
public class AutoLeft extends LinearOpMode {
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

    public static double x = 5;
    public static double y = 0;
    public static double heading = 0;
    public static double approach = 0;

    private TrajectorySequence scoreConeLowPole;
    private Trajectory resetAfterConeLowPole;
    private TrajectorySequence setupForScoringCycle;
    private TrajectorySequence cycleConeStackOnHighPole;
    private TrajectorySequence leftPark;
    private TrajectorySequence rightPark;

    private enum State {
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
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
                .waitSeconds(2)
                .splineToConstantHeading(new Vector2d(3, 0), Math.toRadians(0))
                 .UNSTABLE_addTemporalMarkerOffset(INSTANT_TIME, () -> {
                     liftController.setTargetPosition(LIFT_LEVEL_1);
                 })
                .waitSeconds(2)
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
                .splineToConstantHeading(new Vector2d(50 + x, resetAfterConeLowPole.end().getY() + 2), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50, resetAfterConeLowPole.end().getY() + 2), Math.toRadians(0))
                .turn(Math.toRadians(90))
                .build();

        cycleConeStackOnHighPole = drive.trajectorySequenceBuilder(setupForScoringCycle.end())
                .forward(25)
                .UNSTABLE_addTemporalMarkerOffset(INSTANT_TIME, () -> {
                    servoGrabber.setPower(0);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(INSTANT_TIME, () -> {
                    liftController.setTargetPosition(LIFT_LEVEL_3);
                })
                .splineToSplineHeading(new Pose2d(setupForScoringCycle.end().getX(), 5, Math.toRadians(-90)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(INSTANT_TIME, () -> {
                    servoGrabber.setPower(GRABBER_OPEN_POWER);
                })
                .waitSeconds(0.5)
                .splineToSplineHeading(setupForScoringCycle.end(), Math.toRadians(-90))
                .build();

        leftPark = drive.trajectorySequenceBuilder(cycleConeStackOnHighPole.end())
                .forward(23)
                .turn(-90)
                .build();

        rightPark = drive.trajectorySequenceBuilder(cycleConeStackOnHighPole.end())
                .back(23)
                .turn(-90)
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

        // drive.followTrajectorySequence(scoreConeLowPole);
        // drive.followTrajectory(resetAfterConeLowPole);
        // motorLift.setTargetPosition(LIFT_LEVEL_0);
        // drive.followTrajectorySequence(setupForScoringCycle);

        // int stackPosition = START_STACK_POSITION;

        // for (int i = 0; i < STACK_CYCLES; i++) {
        //     motorLift.setTargetPosition(stackPosition);
        //     drive.followTrajectorySequence(cycleConeStackOnHighPole);

        //     stackPosition -= STACK_POSITION_CONE_OFFSET;
        // }

        // if (parkingLocation == ParkingLocation.LEFT) {
        //     drive.followTrajectorySequence(leftPark);
        // } else if (parkingLocation == ParkingLocation.RIGHT) {
        //     drive.followTrajectorySequence(rightPark);
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
                        currentState = State.IDLE;
                        // if (parkingLocation == ParkingLocation.LEFT) {
                        //     drive.followTrajectorySequence(leftPark);
                        // } else if (parkingLocation == ParkingLocation.RIGHT) {
                        //     drive.followTrajectorySequence(rightPark);
                        // }
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
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
