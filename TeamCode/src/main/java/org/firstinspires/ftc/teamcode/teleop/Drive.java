package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kickstand;
import org.firstinspires.ftc.teamcode.subsystems.Paddle;
import org.firstinspires.ftc.teamcode.subsystems.config.FlywheelConfig;
import org.firstinspires.ftc.teamcode.targeting.AimingCalculator;
import org.firstinspires.ftc.teamcode.targeting.DistanceProvider;
import org.firstinspires.ftc.teamcode.targeting.TeamConfig;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Drive", group = "teleop")
public class Drive extends NextFTCOpMode {

    public static Pose startingPose = new Pose(72, 72, Math.toRadians(90));
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.2;

    private static final double STICK_DEAD_ZONE = 0.07;

    // ------------------- Hold / Input state -------------------
    private Pose aimPose = new Pose();
    private boolean aimRequested = false;

    // Snapshot of where we expected to be when we started aiming
    private Pose aimAnchorPose = new Pose();

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

    // ------------------- Endgame timers -------------------
    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean didRumble145 = false;
    private boolean didShutdown155 = false;

    private static final double RUMBLE_TIME_SEC = 105.0;   // 1:45
    private static final double SHUTDOWN_TIME_SEC = 115.0; // 1:55

    public Drive() {
        addComponents(
                // NextFTC runtime plumbing
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE,
                CommandManager.INSTANCE,

                // Subsystems
                new SubsystemComponent(Flywheel.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(Kickstand.INSTANCE),
                new SubsystemComponent(Paddle.INSTANCE),

                // Pedro integration: creates + updates follower automatically
                new PedroComponent(Constants::createFollower));
    }

    @Override
    public void onInit() {
        PedroComponent.follower().setStartingPose(startingPose == null ? new Pose() : startingPose);

        telemetryM.update(telemetry);
    }

    @Override
    public void onWaitForStart() {
        super.onWaitForStart();

        if (gamepad1.backWasPressed()) {
            TeamConfig.goal = TeamConfig.goal == AimingCalculator.Goal.BLUE_GOAL ?
                    AimingCalculator.Goal.RED_GOAL : AimingCalculator.Goal.BLUE_GOAL;
        }

        telemetryM.debug("goal", TeamConfig.goal);
        telemetryM.update(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        PedroComponent.follower().startTeleopDrive();

        // Start as "input present" so we don't immediately auto-hold on start
        wasDriverInput = true;

        idleHoldActive = false;
        aimHoldActive = false;
        pendingIdleHold = false;

        Flywheel.INSTANCE.enableAutoFromDistance();

        matchTimer.reset();
        didRumble145 = false;
        didShutdown155 = false;
    }

    @Override
    public void onUpdate() {
        telemetryM.update(telemetry);

        DriveInput input = readDriveInput();

        handleDriverInput(input);
        handleIdleSettleAndHold(input);
        wasDriverInput = input.driverInputDetected();

        applyTeleopDrive(input);
        handleAimHoldRequest();
        handleAutoFireWhenAimed();

        // ------------------- Slow Mode toggle --------------------------------
        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }

        // ------------------- Intake ------------------------------------------
        if (gamepad1.aWasPressed()) {
            Intake.INSTANCE.on();
        } else if (gamepad1.bWasPressed()) {
            Intake.INSTANCE.off();
        }

        // ------------------- Flywheel target velocity ------------------------
        if (gamepad1.dpadUpWasPressed()) {
            Flywheel.INSTANCE.setTargetRpm(FlywheelConfig.targetRpm + 50);
        } else if (gamepad1.dpadDownWasPressed()) {
            Flywheel.INSTANCE.setTargetRpm((Math.max(FlywheelConfig.targetRpm - 50, 0)));
        }

        // ------------------- Auto adjust speed for flywheel ------------------
        if (gamepad1.dpadLeftWasPressed()) {
            Flywheel.INSTANCE.disableAutoFromDistance();
        } else if (gamepad1.dpadRightWasPressed()) {
            Flywheel.INSTANCE.enableAutoFromDistance();
        }

        // ------------------- Kickstand ---------------------------------------
        if (gamepad1.xWasPressed()) {
            cancelHolds();
            Kickstand.INSTANCE.deploy();
        } else if (gamepad1.yWasPressed()) {
            Kickstand.INSTANCE.retract();
        }

        double elapsedSec = matchTimer.seconds();

        // Vibrate at 1:45 (once)
        if (!didRumble145 && elapsedSec >= RUMBLE_TIME_SEC) {
            gamepad1.rumbleBlips(5);
            didRumble145 = true;
        }

        // Shutdown at 1:55 (once)
        if (!didShutdown155 && elapsedSec >= SHUTDOWN_TIME_SEC) {
            Flywheel.INSTANCE.stop();
            Intake.INSTANCE.off();
            didShutdown155 = true;
        }

        telemetryM.debug("driverInputDetected", input.driverInputDetected());
        telemetryM.debug("pendingIdleHold", pendingIdleHold);
        telemetryM.debug("idleHoldActive", idleHoldActive);
        telemetryM.debug("aimHoldActive", aimHoldActive);
        telemetryM.debug("target rpm", Flywheel.INSTANCE.getTargetRpm());
        telemetryM.debug("actual rpm", Flywheel.INSTANCE.getCurrentRpm());
        telemetryM.debug("distance from target", DistanceProvider.INSTANCE.getDistance());
        telemetryM.debug("kickstand position", hardwareMap.get(DcMotorEx.class, "kickstand"));
    }

    private DriveInput readDriveInput() {
        return DriveInput.fromRaw(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                STICK_DEAD_ZONE);
    }

    private void handleDriverInput(DriveInput input) {
        if (!input.driverInputDetected()) {
            return;
        }

        // Any stick motion cancels pending idle hold and any pending shot immediately
        pendingIdleHold = false;
        aimRequested = false;

        // If we were previously "no input", this is the transition back to manual
        if (!wasDriverInput) {
            idleHoldActive = false;
            aimHoldActive = false;
            PedroComponent.follower().startTeleopDrive();
        }
    }

    private void handleIdleSettleAndHold(DriveInput input) {
        if (!input.driverInputDetected() && wasDriverInput) {
            pendingIdleHold = true;
            idleSettleTimer.reset();
            PedroComponent.follower().setTeleOpDrive(0, 0, 0, true);
        }

        if (!input.driverInputDetected() && pendingIdleHold) {
            boolean turnIsReallyZero = Math.abs(input.turn()) <= TURN_CANCEL_THRESHOLD;
            if (turnIsReallyZero && idleSettleTimer.seconds() >= IDLE_SETTLE_SEC) {
                Pose idleHoldPose = PedroComponent.follower().getPose();
                idleHoldActive = true;
                aimHoldActive = false;
                pendingIdleHold = false;
                PedroComponent.follower().holdPoint(idleHoldPose);
            }
        }
    }

    private void applyTeleopDrive(DriveInput input) {
        if (!input.driverInputDetected()) {
            return;
        }

        double driveY = input.driveY();
        double driveX = input.driveX();
        double turn = input.turn();

        if (slowMode) {
            driveY *= slowModeMultiplier;
            driveX *= slowModeMultiplier;
            turn *= slowModeMultiplier;
        }

        PedroComponent.follower().setTeleOpDrive(driveY, driveX, turn, true);
    }

    private void handleAimHoldRequest() {
        if (!gamepad1.rightBumperWasPressed()) {
            return;
        }

        Pose currentPose = PedroComponent.follower().getPose();
        aimPose = AimingCalculator.computeAimPose(currentPose, TeamConfig.goal);
        aimRequested = true;

        aimHoldActive = true;
        idleHoldActive = false;
        pendingIdleHold = false;

        // Anchor where we were when we requested the shot
        aimAnchorPose = currentPose;
        PedroComponent.follower().holdPoint(aimPose);
    }

    private void handleAutoFireWhenAimed() {
        if (!aimRequested) {
            return;
        }

        Pose currentPose = PedroComponent.follower().getPose();
        boolean headingGood = angleAbsDiffRad(currentPose.getHeading(), aimPose.getHeading()) <= AIM_HEADING_TOL_RAD;
        boolean anchorStillValid = distanceInches(currentPose, aimAnchorPose) <= AIM_ANCHOR_TOL_IN;

        if (headingGood && anchorStillValid && Flywheel.INSTANCE.isAtSpeed()) {
            CommandManager.INSTANCE.scheduleCommand(Paddle.INSTANCE.feedOnce());
            aimRequested = false;
        }
    }

    private record DriveInput(double driveY, double driveX, double turn, boolean driverInputDetected) {
        private static DriveInput fromRaw(double rawDriveY, double rawDriveX, double rawTurn, double deadZone) {
            double driveY = applyDeadZone(rawDriveY, deadZone);
            double driveX = applyDeadZone(rawDriveX, deadZone);
            double turn = applyDeadZone(rawTurn, deadZone);
            boolean driverInputDetected = (Math.abs(driveY) > 0.0) || (Math.abs(driveX) > 0.0) || (Math.abs(turn) > 0.0);
            return new DriveInput(driveY, driveX, turn, driverInputDetected);
        }

        private static double applyDeadZone(double value, double deadZone) {
            if (Math.abs(value) < deadZone) {
                return 0.0;
            }
            return value;
        }
    }

    @Override
    public void onStop() {
        super.onStop();
        Kickstand.INSTANCE.retract();
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

    private void cancelHolds() {
        pendingIdleHold = false;
        idleHoldActive = false;
        aimHoldActive = false;

        aimRequested = false;

        // Return follower to normal teleop control (cancels holdPoint)
        PedroComponent.follower().startTeleopDrive();
    }

}
