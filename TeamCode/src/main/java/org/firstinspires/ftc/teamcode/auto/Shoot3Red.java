//package org.firstinspires.ftc.teamcode.auto;
//
//import static dev.nextftc.extensions.pedro.PedroComponent.follower;
//
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.LauncherOld;
//import org.firstinspires.ftc.teamcode.teleop.LauncherController;
//
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.commands.delays.Delay;
//import dev.nextftc.core.commands.groups.SequentialGroup;
//import dev.nextftc.core.commands.utility.InstantCommand;
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.extensions.pedro.FollowPath;
//import dev.nextftc.extensions.pedro.PedroComponent;
//import dev.nextftc.ftc.NextFTCOpMode;
//
//@Autonomous(name = "Shoot 3 balls - Red")
//public class Shoot3Red extends NextFTCOpMode {
//
//    public Shoot3Red() {
//        // Register components: our subsystem + Pedro’s follower
//        addComponents(
//                new SubsystemComponent(LauncherOld.INSTANCE),
//                new PedroComponent(Constants::createFollower)
//        );
//
//    }
//
//    // Build a simple straight path to a shooting location
//    private PathChain buildPath() {
//        return follower().pathBuilder()
//                .addPath(
//
//                        new BezierLine(new Pose(0, 0), new Pose(96.5, 0))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-233))
//                .build();
//    }
//
//    // Full autonomous routine
//    private Command autoRoutine() {
//        PathChain toShoot = buildPath();
//        LauncherController launcher = new LauncherController(hardwareMap, telemetry);
//
//        launcher.runSlow();
//
//        return new SequentialGroup(
//                // Make sure follower starts from a known pose if you want
//                new InstantCommand(() ->
//                        follower().setStartingPose(new Pose(0, 0, Math.toRadians(180)))),
//
//                // Drive to the spot and wait until finished following
//                new FollowPath(toShoot),
//                // Spin up + feed two shots with a small cadence, then stop
//                new SequentialGroup(
//                        LauncherOld.INSTANCE.feedOne,
//                        new Delay(1),
//                        LauncherOld.INSTANCE.feedOne,
//                        new Delay(1),
//                        LauncherOld.INSTANCE.feedOne,
//                        new Delay(1),
//                        new InstantCommand(launcher::stop)
//                )
//        );
//    }
//
//
//    @Override
//    public void onStartButtonPressed() {
//        autoRoutine().invoke(); // schedule the routine
//    }
//}
//
//
