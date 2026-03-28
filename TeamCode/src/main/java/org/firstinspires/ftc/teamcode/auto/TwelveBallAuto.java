package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.GoalConfig;
import org.firstinspires.ftc.teamcode.config.GoalSelector;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Paddle;
import org.firstinspires.ftc.teamcode.subsystems.PosePublisher;
import org.firstinspires.ftc.teamcode.subsystems.commands.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.targeting.DistanceProvider;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Shoot 12 balls", group = "auto")
public class TwelveBallAuto extends NextFTCOpMode {

    private static final double AUTO_LENGTH_SEC = 35.0;
    private static final double FLYWHEEL_CUTOFF_REMAINING_SEC = 1.0;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private boolean didFlywheelCutoff = false;

    public final static class AutoPaths {

        private final double shootingAngle = 130; //deg
        private final Pose blueStartingPose = new Pose(55, 8, Math.toRadians(90));
        private final Pose shortShootingPose = new Pose(55, 88, Math.toRadians(shootingAngle));

        public final AutoPathSpec shootPreloadPath = new AutoPathSpec()
                .addLine(blueStartingPose,
                        shortShootingPose)
                .linearHeading(Math.toRadians(90), Math.toRadians(shootingAngle));

        public final AutoPathSpec collect4_6 = new AutoPathSpec()
                .addCurve(
                        shortShootingPose,
                        new Pose(71, 31),
                        new Pose(22, 33)
                )
                .tangentHeading();

        public final AutoPathSpec shoot4_6 = new AutoPathSpec()
                .addCurve(
                        new Pose(22, 33),
                        new Pose(71, 31),
                        shortShootingPose
                )
                .linearHeading(Math.toRadians(180), Math.toRadians(shootingAngle));

        public final AutoPathSpec collect7_9a = new AutoPathSpec()
                .addLine(shortShootingPose,
                        new Pose(47, 60))
                .linearHeading(Math.toRadians(shootingAngle), Math.toRadians(180));

        public final AutoPathSpec collect7_9b = new AutoPathSpec()
                .addLine(new Pose(47, 60),
                        new Pose(22, 58))
                .tangentHeading();

        public final AutoPathSpec shoot7_9 = new AutoPathSpec()
                .addCurve(new Pose(22, 58),
                        new Pose(71, 31),
                        shortShootingPose)
                .linearHeading(Math.toRadians(180), Math.toRadians(shootingAngle));

        public final AutoPathSpec collect10_12 = new AutoPathSpec()
                .addLine(shortShootingPose,
                        new Pose(22, 79))
                .constantHeading(Math.toRadians(180));

        public final AutoPathSpec shoot10_12 = new AutoPathSpec()
                .addLine(new Pose(19, 79),
                        new Pose(57, 100))
                .linearHeading(Math.toRadians(180), Math.toRadians(145));

        public Pose getStartingPose() {
            return FieldMirror.getPose(blueStartingPose, GoalConfig.goal);
        }
    }

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private final PedroComponent pedroComponent = new PedroComponent(AutoConstants::createFollower);

    private Flywheel flywheel;
    private Paddle paddle;
    private Intake intake;
    private Follower follower;
    private Servo light;

    public TwelveBallAuto() {
        // Register components BEFORE init runs
        addComponents(
                pedroComponent
        );
    }

    @Override
    public void onInit() {
        super.onInit();

        light = hardwareMap.get(Servo.class, "light");

        // Now PedroComponent has been initialized -> follower exists
        follower = PedroComponent.follower();

        paddle = new Paddle();
        intake = new Intake();

        DistanceProvider distanceProvider = new DistanceProvider(follower);
        flywheel = new Flywheel(distanceProvider);

        // If your SubsystemComponent needs the real subsystem instances,
        // create it here instead of in the constructor:
        addComponents(
                new SubsystemComponent(
                        flywheel,
                        paddle,
                        intake,
                        new PosePublisher(follower)
                )
        );

        paddle.lower.run();
    }

    @Override
    public void onStartButtonPressed() {
        autoTimer.reset();
        didFlywheelCutoff = false;

        autoRoutine().invoke();
    }

    @Override
    public void onUpdate() {
        super.onUpdate();

        double elapsed = autoTimer.seconds();
        double remaining = AUTO_LENGTH_SEC - elapsed;

        telemetryM.addData("remaining", remaining);
        telemetryM.update();

        if (!didFlywheelCutoff && remaining <= FLYWHEEL_CUTOFF_REMAINING_SEC) {
            flywheel.stop();
            intake.off();
            didFlywheelCutoff = true;
        }
    }

    private Command autoRoutine() {
        AutoPaths paths = new AutoPaths();

        follower.setStartingPose(paths.getStartingPose());

        return new SequentialGroup(
                paddle.lower,

                // Enable distance-based flywheel RPM
                new InstantCommand(flywheel::enableAutoFromDistance),

                // Move forward to shoot preloaded balls
                new FollowPath(paths.shootPreloadPath.build(follower, GoalConfig.goal)),

                // Shoot preloaded balls
                shootCommand(),
                new InstantCommand(intake::on),
                new Delay(0.25),
                shootCommand(),
                new Delay(0.25),
                shootCommand(),

                // Collect balls 4-6 and drive to shooting position
                new FollowPath(paths.collect4_6.build(follower, GoalConfig.goal)),
                new InstantCommand(intake::off),

                // Drive to shooting location
                new FollowPath(paths.shoot4_6.build(follower, GoalConfig.goal)),

                // Shoot balls 3-6
                shootCommand(),
                new Delay(0.25),
                new InstantCommand(intake::on),
                shootCommand(),
                new Delay(0.25),
                shootCommand(),

                //Collect balls 7-9
                new FollowPath(paths.collect7_9a.build(follower, GoalConfig.goal)),
                new FollowPath(paths.collect7_9b.build(follower, GoalConfig.goal)),
                new InstantCommand(intake::off),

                // Drive to shooting location
                new FollowPath(paths.shoot7_9.build(follower, GoalConfig.goal)),

                //Shoot balls 7-9
                shootCommand(),
                new Delay(0.25),
                new InstantCommand(intake::on),
                shootCommand(),
                new Delay(0.25),
                shootCommand(),

                //Collect balls 10-12
                new FollowPath(paths.collect10_12.build(follower, GoalConfig.goal)),
                new InstantCommand(intake::off),

                // Drive to shooting location
                new FollowPath(paths.shoot10_12.build(follower, GoalConfig.goal)),

                // shoot balls 10-12
                shootCommand(),
                new Delay(0.25),
                new InstantCommand(intake::on),
                shootCommand(),
                new Delay(0.25),
                shootCommand(),

                // Stop flywheel after shooting
                new InstantCommand(flywheel::stop),
                new InstantCommand(intake::off)
        );
    }

    private Command shootCommand() {
        if (!didFlywheelCutoff) {
            return new SequentialGroup(
                    new WaitUntilCommand(flywheel::isAtSpeed),
                    paddle.feedOnce());
        }

        return new InstantCommand(() -> {
            // do nothing; time expired
        });
    }

    @Override
    public void onWaitForStart() {
        super.onWaitForStart();
        GoalSelector.update(gamepad1, light, telemetryM);
        telemetryM.update(telemetry);
    }
}
