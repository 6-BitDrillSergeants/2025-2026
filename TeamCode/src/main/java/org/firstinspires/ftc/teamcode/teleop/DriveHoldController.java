package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Paddle;
import org.firstinspires.ftc.teamcode.targeting.AimingCalculator;
import org.firstinspires.ftc.teamcode.targeting.TeamConfig;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.extensions.pedro.PedroComponent;

final class DriveHoldController {

    private static final double IDLE_SETTLE_SEC = 0.15;
    private static final double TURN_CANCEL_THRESHOLD = 0.02;

    private static final double AIM_HEADING_TOL_RAD = Math.toRadians(2.0);
    private static final double AIM_ANCHOR_TOL_IN = 2.0;

    private Pose aimPose = new Pose();
    private Pose aimAnchorPose = new Pose();

    private boolean aimRequested = false;
    private boolean wasDriverInput = true;
    private boolean idleHoldActive = false;
    private boolean aimHoldActive = false;
    private boolean pendingIdleHold = false;

    private final ElapsedTime idleSettleTimer = new ElapsedTime();

    void resetForStart() {
        PedroComponent.follower().startTeleopDrive();

        wasDriverInput = true;
        idleHoldActive = false;
        aimHoldActive = false;
        pendingIdleHold = false;
        aimRequested = false;
    }

    void handleDriverInput(boolean driverInputDetected) {
        if (!driverInputDetected) {
            return;
        }

        pendingIdleHold = false;
        aimRequested = false;

        if (!wasDriverInput) {
            idleHoldActive = false;
            aimHoldActive = false;
            PedroComponent.follower().startTeleopDrive();
        }
    }

    void handleIdleSettleAndHold(boolean driverInputDetected, double turnInput) {
        if (!driverInputDetected && wasDriverInput) {
            pendingIdleHold = true;
            idleSettleTimer.reset();
            PedroComponent.follower().setTeleOpDrive(0, 0, 0, true);
        }

        if (!driverInputDetected && pendingIdleHold) {
            boolean turnIsReallyZero = Math.abs(turnInput) <= TURN_CANCEL_THRESHOLD;
            if (turnIsReallyZero && idleSettleTimer.seconds() >= IDLE_SETTLE_SEC) {
                Pose idleHoldPose = PedroComponent.follower().getPose();
                idleHoldActive = true;
                aimHoldActive = false;
                pendingIdleHold = false;
                PedroComponent.follower().holdPoint(idleHoldPose);
            }
        }
    }

    void updateDriverInputState(boolean driverInputDetected) {
        wasDriverInput = driverInputDetected;
    }

    void handleAimHoldRequest(boolean aimButtonPressed) {
        if (!aimButtonPressed) {
            return;
        }

        Pose currentPose = PedroComponent.follower().getPose();
        aimPose = AimingCalculator.computeAimPose(currentPose, TeamConfig.goal);
        aimRequested = true;

        aimHoldActive = true;
        idleHoldActive = false;
        pendingIdleHold = false;

        aimAnchorPose = currentPose;
        PedroComponent.follower().holdPoint(aimPose);
    }

    void handleAutoFireWhenAimed() {
        if (!aimRequested) {
            return;
        }

        Pose currentPose = PedroComponent.follower().getPose();
        boolean headingGood = isHeadingGood(currentPose);
        boolean anchorStillValid = isAnchorGood(currentPose);

        if (headingGood && anchorStillValid && Flywheel.INSTANCE.isAtSpeed()) {
            CommandManager.INSTANCE.scheduleCommand(Paddle.INSTANCE.feedOnce());
            aimRequested = false;
        }
    }

    void cancelHolds() {
        pendingIdleHold = false;
        idleHoldActive = false;
        aimHoldActive = false;
        aimRequested = false;

        PedroComponent.follower().startTeleopDrive();
    }

    boolean isHeadingGood(Pose currentPose) {
        return angleAbsDiffRad(currentPose.getHeading(), aimPose.getHeading()) <= AIM_HEADING_TOL_RAD;
    }

    boolean isAnchorGood(Pose currentPose) {
        return distanceInches(currentPose, aimAnchorPose) <= AIM_ANCHOR_TOL_IN;
    }

    String activeDriveMode() {
        if (aimHoldActive) {
            return "AIM_HOLD";
        }
        if (idleHoldActive) {
            return "IDLE_HOLD";
        }
        if (pendingIdleHold) {
            return "SETTLING";
        }
        return "DRIVER";
    }

    String firstBlockingCondition(boolean flywheelReady, Pose currentPose) {
        if (!aimRequested) {
            return "Waiting for aim request";
        }
        if (!isHeadingGood(currentPose)) {
            return "Heading not in tolerance";
        }
        if (!isAnchorGood(currentPose)) {
            return "Robot moved too far while aiming";
        }
        if (!flywheelReady) {
            return "Flywheel not at speed";
        }
        return "Ready to fire";
    }

    boolean isAimRequested() {
        return aimRequested;
    }

    boolean isIdleHoldActive() {
        return idleHoldActive;
    }

    boolean isAimHoldActive() {
        return aimHoldActive;
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
