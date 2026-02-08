package org.firstinspires.ftc.teamcode.targeting;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.config.GoalConfig;

import dev.nextftc.extensions.pedro.PedroComponent;

public final class DistanceProvider {

    public static final DistanceProvider INSTANCE = new DistanceProvider();

    private DistanceProvider() {
    }

    /**
     * Returns distance from current pose to (targetX, targetY).
     */
    public double getDistance() {
        Follower follower = PedroComponent.follower();

        Pose current = follower.getPose();
        Pose target = AimingCalculator.computeDynamicGoalPose(current, GoalConfig.goal);

        double dx = target.getX() - current.getX();
        double dy = target.getY() - current.getY();
        return Math.hypot(dx, dy);
    }
}
