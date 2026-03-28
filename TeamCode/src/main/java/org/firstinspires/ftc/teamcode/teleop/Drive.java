package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.GoalSelector;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleopConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kickstand;
import org.firstinspires.ftc.teamcode.subsystems.Paddle;
import org.firstinspires.ftc.teamcode.subsystems.config.FlywheelConfig;
import org.firstinspires.ftc.teamcode.targeting.DistanceProvider;

import java.util.Locale;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Drive", group = "teleop")
public class Drive extends NextFTCOpMode {

    public static Pose startingPose;
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private boolean slowMode = false;

    //private static final double STICK_DEAD_ZONE = .05;

    private DriveHoldController holdController;

    // ------------------- Endgame timers -------------------
    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean didRumble145 = false;
    private boolean didShutdown155 = false;

    private static final double RUMBLE_TIME_SEC = 105.0;   // 1:45
    private static final double SHUTDOWN_TIME_SEC = 115.0; // 1:55 is 115.0, has been increased for testing

    private final PedroComponent pedroComponent = new PedroComponent(TeleopConstants::createFollower);

    private Flywheel flywheel;
    private Paddle paddle;
    private Intake intake;
    private Kickstand kickstand;
    private Follower follower;
    private DistanceProvider distanceProvider;
    private Servo light;

    public Drive() {
        // Register components BEFORE init runs
        addComponents(
                pedroComponent
        );
    }

    @Override
    public void onInit() {
        super.onInit();


        light = hardwareMap.get(Servo.class, "light");

        intake = new Intake();
        paddle = new Paddle();
        kickstand = new Kickstand();
        follower = PedroComponent.follower();
        distanceProvider = new DistanceProvider(follower);
        flywheel = new Flywheel(distanceProvider);
        holdController = new DriveHoldController(follower, flywheel, paddle);

        addComponents(
                // NextFTC runtime plumbing
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE,
                CommandManager.INSTANCE,

                // Subsystems
                new SubsystemComponent(
                        flywheel,
                        intake,
                        kickstand,
                        paddle));
    }

    @Override
    public void onWaitForStart() {
        super.onWaitForStart();
        GoalSelector.update(gamepad1, light, telemetryM);
        telemetryM.update(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        startingPose = RobotConfig.getCurrentPose();
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        holdController.start();

        flywheel.enableAutoFromDistance();
        paddle.lower.run();

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
        if (gamepad1.leftTriggerWasPressed()) {
            if (intake.isOn()) {
                intake.off();
            } else {
                intake.on();
            }
        }

        // ------------------- Flywheel target velocity ------------------------
        if (gamepad1.dpadUpWasPressed()) {
            flywheel.setTargetRpm(FlywheelConfig.targetRpm + 50);
        } else if (gamepad1.dpadDownWasPressed()) {
            flywheel.setTargetRpm((Math.max(FlywheelConfig.targetRpm - 50, 0)));
        }

        // ------------------- Auto adjust speed for flywheel ------------------
        if (gamepad1.dpadLeftWasPressed()) {
            flywheel.disableAutoFromDistance();
        } else if (gamepad1.dpadRightWasPressed()) {
            flywheel.enableAutoFromDistance();
        }

        // ------------------- Force shot --------------------------------------
        if (gamepad1.rightTriggerWasPressed()) {
            paddle.feedOnce().run();
        }

        // ------------------- Kickstand ---------------------------------------
        if (gamepad1.xWasPressed()) {
            holdController.cancelHolds();
            kickstand.deploy();
        } else if (gamepad1.yWasPressed()) {
            kickstand.retract();
        }

        double elapsedSec = matchTimer.seconds();

        // Vibrate at 1:45 (once)
        if (!didRumble145 && elapsedSec >= RUMBLE_TIME_SEC) {
            gamepad1.rumbleBlips(5);
            didRumble145 = true;
        }

        // Shutdown at 1:55 (once) UPDATED TO 500 SECONDS
        if (!didShutdown155 && elapsedSec >= SHUTDOWN_TIME_SEC) {
            flywheel.stop();
            intake.off();
            didShutdown155 = true;
        }

        publishCompetitionTelemetry(input);
    }

    private DriveInput readDriveInput() {
        return DriveInput.fromRaw(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x);
    }


    private void applyTeleopDrive(DriveInput input) {
        if (!input.driverInputDetected()) {
            return;
        }

        double driveY = input.driveY();
        double driveX = input.driveX();
        double turn = input.turn();

        if (slowMode) {
            double slowModeMultiplier = 0.2;
            driveY *= slowModeMultiplier;
            driveX *= slowModeMultiplier;
            turn *= slowModeMultiplier;
        }

        follower.setTeleOpDrive(driveY, driveX, turn, true);
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

        private static DriveInput fromRaw(double rawDriveY, double rawDriveX, double rawTurn) {
            double driveY = applyDeadZone(rawDriveY);
            double driveX = applyDeadZone(rawDriveX);
            double turn = applyDeadZone(rawTurn);
            boolean driverInputDetected = (Math.abs(driveY) > 0.0) || (Math.abs(driveX) > 0.0) || (Math.abs(turn) > 0.0);
            return new DriveInput(driveY, driveX, turn, driverInputDetected);
        }

        private static double applyDeadZone(double value) {
            /*if (Math.abs(value) < Drive.STICK_DEAD_ZONE) {
                return 0.0;
            }*/
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
        kickstand.retract();
    }

    private void publishCompetitionTelemetry(DriveInput input) {
        Pose currentPose = follower.getPose();
        double distanceToGoal = distanceProvider.getDistance();
        double targetRpm = flywheel.getTargetRpm();
        double currentRpm = flywheel.getCurrentRpm();
        double rpmError = targetRpm - currentRpm;

        boolean headingGood = holdController.isHeadingGood(currentPose);
        boolean anchorGood = holdController.isAnchorGood(currentPose);
        boolean flywheelReady = flywheel.isAtSpeed();

        telemetryM.debug("=== DRIVE ===");
        telemetryM.debug(String.format(
                Locale.US,
                "Mode: %s  Slow: %s  Input: %s",
                holdController.activeDriveMode(),
                asStatus(slowMode),
                asStatus(input.driverInputDetected())));

        telemetryM.debug(String.format(
                Locale.US,
                "Pose: X:%6.2f  Y:%6.2f  H:%6.1f°",
                currentPose.getX(),
                currentPose.getY(),
                Math.toDegrees(currentPose.getHeading())));

        if (holdController.isAimRequested()) {
            Pose aimPose = holdController.getAimPose();
            telemetryM.debug(String.format(
                    Locale.US,
                    "Aim:  X:%6.2f  Y:%6.2f  H:%6.1f°",
                    aimPose.getX(),
                    aimPose.getY(),
                    Math.toDegrees(aimPose.getHeading())));
        }

        telemetryM.debug("");
        telemetryM.debug("=== LAUNCHER ===");
        telemetryM.debug(String.format(
                Locale.US,
                "Shot Distance: %5.1fin",
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
        telemetryM.debug("=== Intake ===");
        telemetryM.debug("Intake Power: " + hardwareMap.get(DcMotorEx.class, "intake").getPower());


        telemetryM.debug("");
        telemetryM.debug("=== KICKSTAND ===");
        telemetryM.debug("Kickstand position: " + hardwareMap.get(DcMotorEx.class, "kickstand").getCurrentPosition());

        telemetryM.debug("");
        telemetryM.debug("*** END OF DEBUG ***");
        telemetryM.debug("");
        flywheel.publishTelemetry(telemetryM);
    }

    private String asStatus(boolean ready) {
        return ready ? "[✓]" : "[   ]";
    }
}
