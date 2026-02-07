package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
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

import java.util.Locale;

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

    private final DriveHoldController holdController = new DriveHoldController();

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

        // Monitor "back" button to select team goal
        if (gamepad1.backWasPressed()) {
            TeamConfig.goal = TeamConfig.goal == AimingCalculator.Goal.BLUE_GOAL ?
                    AimingCalculator.Goal.RED_GOAL : AimingCalculator.Goal.BLUE_GOAL;
        }

        // Set controller LED to match goal color
        if (TeamConfig.goal == AimingCalculator.Goal.RED_GOAL) {
            gamepad1.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS); // red
        } else if (TeamConfig.goal == AimingCalculator.Goal.BLUE_GOAL) {
            gamepad1.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS); // blue
        }

        telemetryM.debug("Use 'back' button to select goal", TeamConfig.goal);
        telemetryM.debug("");
        telemetryM.debug("Goal: " + telemetry);
        telemetryM.update(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        holdController.resetForStart();

        Flywheel.INSTANCE.enableAutoFromDistance();

        matchTimer.reset();
        didRumble145 = false;
        didShutdown155 = false;
    }

    @Override
    public void onUpdate() {
        telemetryM.update(telemetry);

        DriveInput input = readDriveInput();

        holdController.handleDriverInput(input.driverInputDetected());
        holdController.handleIdleSettleAndHold(input.driverInputDetected(), input.turn());
        holdController.updateDriverInputState(input.driverInputDetected());

        applyTeleopDrive(input);
        holdController.handleAimHoldRequest(gamepad1.rightBumperWasPressed());
        holdController.handleAutoFireWhenAimed();

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
            holdController.cancelHolds();
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

        publishCompetitionTelemetry(input, elapsedSec);
    }

    private DriveInput readDriveInput() {
        return DriveInput.fromRaw(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                STICK_DEAD_ZONE);
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



    private static final class DriveInput {
        private final double driveY;
        private final double driveX;
        private final double turn;
        private final boolean driverInputDetected;

        private DriveInput(double driveY, double driveX, double turn, boolean driverInputDetected) {
            this.driveY = driveY;
            this.driveX = driveX;
            this.turn = turn;
            this.driverInputDetected = driverInputDetected;
        }

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

        private double driveY() {
            return driveY;
        }

        private double driveX() {
            return driveX;
        }

        private double turn() {
            return turn;
        }

        private boolean driverInputDetected() {
            return driverInputDetected;
        }
    }

    @Override
    public void onStop() {
        super.onStop();
        Kickstand.INSTANCE.retract();
    }





    private void publishCompetitionTelemetry(DriveInput input, double elapsedSec) {
        Pose currentPose = PedroComponent.follower().getPose();
        double distanceToGoal = DistanceProvider.INSTANCE.getDistance();
        double targetRpm = Flywheel.INSTANCE.getTargetRpm();
        double currentRpm = Flywheel.INSTANCE.getCurrentRpm();
        double rpmError = targetRpm - currentRpm;

        boolean headingGood = holdController.isHeadingGood(currentPose);
        boolean anchorGood = holdController.isAnchorGood(currentPose);
        boolean flywheelReady = Flywheel.INSTANCE.isAtSpeed();

        telemetryM.debug("=== DRIVE ===");
        telemetryM.debug(String.format(
                Locale.US,
                "Mode: %s  Slow: %s  Input: %s",
                holdController.activeDriveMode(),
                asStatus(slowMode),
                asStatus(input.driverInputDetected())));

        telemetryM.debug(String.format(
                Locale.US,
                "Pose X:%6.2f  Y:%6.2f  H:%6.1f°",
                currentPose.getX(),
                currentPose.getY(),
                Math.toDegrees(currentPose.getHeading())));

        telemetryM.debug("");
        telemetryM.debug("=== LAUNCHER ===");
        telemetryM.debug(String.format(
                Locale.US,
                "Distance from target: %5.1fin",
                distanceToGoal));
        telemetryM.debug(String.format(
                Locale.US,
                "RPM: %4.0f/%4.0f  Err: %+5.0f",
                currentRpm,
                targetRpm,
                rpmError));

        telemetryM.debug("");

        telemetryM.debug(String.format(
                Locale.US,
                "%s Aim requested    %s Flywheel speed",
                asStatus(holdController.isAimRequested()),
                asStatus(flywheelReady)));
        telemetryM.debug(String.format(
                Locale.US,
                "%s Heading                %s Anchor",
                asStatus(headingGood),
                asStatus(anchorGood)));

        telemetryM.debug("");
        telemetryM.debug("Blocked by: " + holdController.firstBlockingCondition(flywheelReady, currentPose));

        telemetryM.debug("");
        telemetryM.debug("Holds:");
        telemetryM.debug(String.format("%s Idle                %s Aiming",
                asStatus(holdController.isIdleHoldActive()),
                asStatus(holdController.isAimHoldActive())));

        telemetryM.debug("");
        telemetryM.debug("=== KICKSTAND ===");
        telemetryM.debug("Kickstand position: " + hardwareMap.get(DcMotorEx.class, "kickstand").getCurrentPosition());
    }

    private String asStatus(boolean ready) {
        return ready ? "[✓]" : "[   ]";
    }

    private String formatMatchTime(double elapsedSec) {
        int totalSeconds = (int) Math.max(0, Math.floor(elapsedSec));
        int minutes = totalSeconds / 60;
        int seconds = totalSeconds % 60;
        return String.format(Locale.US, "%d:%02d", minutes, seconds);
    }

}
