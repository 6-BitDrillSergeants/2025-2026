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

@Autonomous(name = "Shoot From Back", group = "auto")
public class FarBackAuto extends NextFTCOpMode {

    private static final double AUTO_LENGTH_SEC = 35.0;
    private static final double FLYWHEEL_CUTOFF_REMAINING_SEC = 1.0;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private boolean didFlywheelCutoff = false;

    public final static class AutoPaths {

        private final double shootingAngle = 109; //deg
        private final Pose blueStartingPose = new Pose(55, 8, Math.toRadians(90));
        private final Pose shortShootingPose = new Pose(55, 15, Math.toRadians(shootingAngle));

        private final Pose collectBackPose = new Pose(7,8, Math.toRadians(180));

        public final AutoPathSpec shootPreloadPath = new AutoPathSpec()
                .addLine(blueStartingPose,
                        shortShootingPose)
                .linearHeading(Math.toRadians(90), Math.toRadians(shootingAngle));

        public final AutoPathSpec goGrab = new AutoPathSpec()
                .addLine(shortShootingPose,
                        collectBackPose)
                .constantHeading(Math.toRadians(180));

        public final AutoPathSpec returnToShoot = new AutoPathSpec()
                .addLine(collectBackPose,
                        shortShootingPose)
                .linearHeading(Math.toRadians(180),Math.toRadians(shootingAngle));

        public final AutoPathSpec offLine = new AutoPathSpec()
                .addLine(shortShootingPose,
                        new Pose(55,30))
                .linearHeading(Math.toRadians(shootingAngle), Math.toRadians(90));

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


    public FarBackAuto() {
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

        // Enable far back flywheel RPM
        flywheel.setTargetRpm(3600);
        return new SequentialGroup(
                paddle.lower,

                // Move forward to shoot preloaded balls
                new FollowPath(paths.shootPreloadPath.build(follower, GoalConfig.goal)),

                // Shoot preloaded balls
                shootCommand(),
                new InstantCommand(intake::on),
                new Delay(0.25),
                shootCommand(),
                new Delay(0.25),
                shootCommand(),

                // Collect balls
                new FollowPath(paths.goGrab.build(follower, GoalConfig.goal)),
                new Delay(0.25),
                new InstantCommand(intake::off),

                // Drive to shooting location
                new FollowPath(paths.returnToShoot.build(follower, GoalConfig.goal)),

                // Shoot balls 3-6
                shootCommand(),
                new Delay(0.25),
                new InstantCommand(intake::on),
                shootCommand(),
                new Delay(0.25),
                shootCommand(),

                //Collect balls
                new FollowPath(paths.goGrab.build(follower, GoalConfig.goal)),
                new Delay(0.25),
                new InstantCommand(intake::off)



                // Drive to shooting location
//                new FollowPath(paths.returnToShoot.build(follower, GoalConfig.goal)),
//
//                //Shoot balls 7-9
//                shootCommand(),
//                new Delay(0.25),
//                new InstantCommand(intake::on),
//                shootCommand(),
//                new Delay(0.25),
//                shootCommand(),
//
//                //Collect balls
//                new FollowPath(paths.goGrab.build(follower, GoalConfig.goal)),
//                new Delay(0.25),
//                new InstantCommand(intake::off)


        );
    }

    private Command shootCommand() {
        if (!didFlywheelCutoff) {
            return new SequentialGroup(
                    new WaitUntilCommand(flywheel::isAtSpeed),
                    paddle.feedOnce(intake));
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
