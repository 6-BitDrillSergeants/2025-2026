package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

import dev.nextftc.core.subsystems.Subsystem;

public final class PosePublisher implements Subsystem {

    public static final PosePublisher INSTANCE = new PosePublisher();

    private PosePublisher() {
    }

    @Override
    public void initialize() {
        Subsystem.super.initialize();
        Drawing.init();
    }

    @Override
    public void periodic() {
        Pose pose = follower().getPose();
        RobotConfig.setCurrentPose(pose);
        Drawing.drawDebug(follower());  // robot + path + history
    }
}
