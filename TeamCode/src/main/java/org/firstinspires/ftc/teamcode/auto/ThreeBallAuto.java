package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

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

@Autonomous(name = "Shoot 3 balls", group = "auto")
public class ThreeBallAuto extends NextFTCOpMode {
    public final static class AutoPaths {

        private final double shootingAngle = 150; //deg
        private final Pose blueStartingPose = new Pose(12.8, 110.8, Math.toRadians(90));
        private final Pose shortShootingPose = new Pose(55, 110.8, Math.toRadians(shootingAngle));

        public final AutoPathSpec shootPreloadPath = new AutoPathSpec()
                .addLine(blueStartingPose,
                        shortShootingPose)
                .linearHeading(Math.toRadians(90), Math.toRadians(shootingAngle));

        public Pose getStartingPose() {
            return FieldMirror.getPose(blueStartingPose, GoalConfig.goal);
        }
    }

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private Servo light;
    private final PedroComponent pedroComponent = new PedroComponent(AutoConstants::createFollower);

    private Flywheel flywheel;
    private Paddle paddle;
    private Intake intake;
    private Follower follower;

    public ThreeBallAuto() {
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
                new Delay(1.00),
                shootCommand(),
                new Delay(1.00),
                shootCommand(),

                // Stop flywheel after shooting
                new InstantCommand(flywheel::stop),
                new InstantCommand(intake::off)
        );
    }

    private SequentialGroup shootCommand() {
        return new SequentialGroup(
                new WaitUntilCommand(flywheel::isAtSpeed),
                paddle.feedOnce());
    }

    @Override
    public void onWaitForStart() {
        super.onWaitForStart();
        GoalSelector.update(gamepad1, light, telemetryM);
        telemetryM.update(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        autoRoutine().invoke();
    }
}
