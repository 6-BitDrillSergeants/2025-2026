package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pidTuning.LauncherFlywheelController;
import org.firstinspires.ftc.teamcode.pidTuning.LauncherFlywheelTuning;
import org.firstinspires.ftc.teamcode.subsystems.Paddle;
import org.firstinspires.ftc.teamcode.targeting.AimingCalculator;

import dev.nextftc.core.commands.CommandManager;

@TeleOp
public class Drive extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(72, 72, Math.toRadians(90));

    private boolean automatedDrive;
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private DcMotorEx intakeMotor;
    private DcMotorEx flywheelMotor;
    private Servo paddleServo;
    private LauncherFlywheelController controller;

    private DcMotorEx kickstandMotor;

    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.2;

    private static final double STICK_DEAD_ZONE = 0.07;

    // ------------------- Hold / Input state -------------------
    private Pose targetPose = new Pose();
    private boolean aimRequested = false;

    // Snapshot of where we expected to be when we started aiming
    private Pose aimAnchorPose = new Pose();
    private Pose idleHoldPose = new Pose();

    private boolean wasDriverInput = true;

    private boolean idleHoldActive = false;
    private boolean aimHoldActive = false;

    private boolean pendingIdleHold = false;
    private final ElapsedTime idleSettleTimer = new ElapsedTime();

    // Tuning knobs
    private static final double IDLE_SETTLE_SEC = 0.15;          // delay before capturing idle pose
    private static final double TURN_CANCEL_THRESHOLD = 0.02;    // "really zero" turn after deadzone

    private static final double AIM_HEADING_TOL_RAD = Math.toRadians(2.0); // 1–3 deg typical
    private static final double AIM_ANCHOR_TOL_IN = 2.0;                   // inches drift allowed
    private static final double AIM_HEADING_DRIFT_TOL_RAD = Math.toRadians(3.0); // extra safety
    // ----------------------------------------------------------

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM.update(telemetry);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        paddleServo = hardwareMap.get(Servo.class, "paddleServo");
        paddleServo.setPosition(.58);

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new LauncherFlywheelController(flywheelMotor);

        kickstandMotor = hardwareMap.get(DcMotorEx.class, "kickstandMotor");
        kickstandMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kickstandMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

        // Start as "input present" so we don't immediately auto-hold on start
        wasDriverInput = true;

        idleHoldActive = false;
        aimHoldActive = false;
        pendingIdleHold = false;
    }

    @Override
    public void loop() {
        controller.update();
        follower.update();
        telemetryM.update(telemetry);

        // Read sticks with deadzone first
        double driveY = applyDeadZone(-gamepad1.left_stick_y, STICK_DEAD_ZONE);
        double driveX = applyDeadZone(-gamepad1.left_stick_x, STICK_DEAD_ZONE);
        double turn = applyDeadZone(-gamepad1.right_stick_x, STICK_DEAD_ZONE);

        boolean driverInputDetected = (Math.abs(driveY) > 0.0) || (Math.abs(driveX) > 0.0) || (Math.abs(turn) > 0.0);

        // ------------------- Cancel holds on driver input -------------------
        if (driverInputDetected) {
            // Any stick motion cancels pending idle hold immediately
            pendingIdleHold = false;

            // If we were previously "no input", then this is the transition back to manual
            if (!wasDriverInput) {
                idleHoldActive = false;
                aimHoldActive = false;

                // Cancel hold by returning to teleop drive mode
                follower.startTeleopDrive();
            }
        }

        // ------------------- Begin idle settle when sticks go neutral --------
        if (!driverInputDetected && wasDriverInput && !automatedDrive) {
            pendingIdleHold = true;
            idleSettleTimer.reset();

            follower.setTeleOpDrive(0, 0, 0, true // Robot Centric
            );
        }

        // ------------------- Commit idle hold after settle delay -------------
        if (!driverInputDetected && pendingIdleHold && !automatedDrive) {
            boolean turnIsReallyZero = Math.abs(turn) <= TURN_CANCEL_THRESHOLD;

            if (turnIsReallyZero && idleSettleTimer.seconds() >= IDLE_SETTLE_SEC) {
                idleHoldPose = follower.getPose(); // capture AFTER settle
                idleHoldActive = true;
                aimHoldActive = false;
                pendingIdleHold = false;

                follower.holdPoint(idleHoldPose);
            }
        }

        wasDriverInput = driverInputDetected;

        // ------------------- TeleOp drive when driver is commanding ----------
        if (!automatedDrive && driverInputDetected) {
            if (slowMode) {
                driveY *= slowModeMultiplier;
                driveX *= slowModeMultiplier;
                turn *= slowModeMultiplier;
            }

            follower.setTeleOpDrive(driveY, driveX, turn, true);
        }

        // ------------------- Aim hold (right bumper) -------------------------
        // Note: This will still be canceled automatically on next stick input.
        if (gamepad1.rightBumperWasPressed()) {
            targetPose = AimingCalculator.computeAimPose(
                    follower.getPose(),
                    AimingCalculator.Goal.BLUE_GOAL
            );

            aimRequested = true;

            aimHoldActive = true;
            idleHoldActive = false;
            pendingIdleHold = false;

            // Anchor where we were when we requested the shot
            aimAnchorPose = follower.getPose();

            follower.holdPoint(targetPose);
        }

        if (aimRequested) {
            Pose currentPose = follower.getPose();

            boolean headingGood =
                    angleAbsDiffRad(currentPose.getHeading(), targetPose.getHeading()) <= AIM_HEADING_TOL_RAD;

            boolean anchorStillValid =
                    distanceInches(currentPose, aimAnchorPose) <= AIM_ANCHOR_TOL_IN &&
                            angleAbsDiffRad(currentPose.getHeading(), aimAnchorPose.getHeading()) <= AIM_HEADING_DRIFT_TOL_RAD;

            if (headingGood && anchorStillValid) {
                // We are aimed and we haven't been bumped since aiming began
                CommandManager.INSTANCE.scheduleCommand(Paddle.INSTANCE.feedOnce());

                aimRequested = false; // consume request so it only fires once
            }
        }

        // ------------------- Slow Mode toggle --------------------------------
        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }

        // ------------------- Intake ------------------------------------------
        if (gamepad1.aWasPressed()) {
            intakeMotor.setPower(1);
        } else if (gamepad1.bWasPressed()) {
            intakeMotor.setPower(0);
        }

        // ------------------- Flywheel target velocity ------------------------
        if (gamepad1.dpadUpWasPressed()) {
            LauncherFlywheelTuning.targetVelocity += 20;
        } else if (gamepad1.dpadDownWasPressed()) {
            LauncherFlywheelTuning.targetVelocity -= 20;
        }

        // ------------------- Kickstand (if enabled) --------------------------
        if (gamepad1.xWasPressed()) {
            if (kickstandMotor != null) {
                kickstandMotor.setTargetPosition(800);
                kickstandMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                kickstandMotor.setPower(1);
            }
        }

        telemetryM.debug("driverInputDetected", driverInputDetected);
        telemetryM.debug("pendingIdleHold", pendingIdleHold);
        telemetryM.debug("idleHoldActive", idleHoldActive);
        telemetryM.debug("aimHoldActive", aimHoldActive);
        telemetryM.debug("current pose", follower.getPose());
        telemetryM.debug("target pose", targetPose);
        telemetryM.debug("idle hold pose", idleHoldPose);
    }

    private double applyDeadZone(double value, double deadZone) {
        if (Math.abs(value) < deadZone) {
            return 0.0;
        }
        return value;
    }

    private static double angleAbsDiffRad(double a, double b) {
        double diff = a - b;
        while (diff > Math.PI) diff -= 2.0 * Math.PI;
        while (diff < -Math.PI) diff += 2.0 * Math.PI;
        return Math.abs(diff);
    }

    private static double distanceInches(Pose a, Pose b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }
}
