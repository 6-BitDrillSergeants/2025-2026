package org.firstinspires.ftc.teamcode.targeting;

import com.pedropathing.geometry.Pose;

public final class AimingCalculator {

    // ------------------------------------------------------------
    // GOAL ENUM
    // ------------------------------------------------------------

    public enum Goal {
        BLUE_GOAL,   // (0, 144)
        RED_GOAL     // (144, 144)
    }

    // ------------------------------------------------------------
    // BASE GOAL COORDINATES
    // ------------------------------------------------------------

    private static final double BLUE_GOAL_X = 0.0;
    private static final double RED_GOAL_X = 144.0;
    private static final double GOAL_Y = 144.0;

    // ------------------------------------------------------------
    // OFFSET CONFIGURATION
    // ------------------------------------------------------------

    private static final double FIELD_CENTER = 72.0;
    private static final double OFFSET_MAX = 12.0;

    private AimingCalculator() {
        // static utility
    }

    // ------------------------------------------------------------
    // PUBLIC API
    // ------------------------------------------------------------

    /**
     * Returns a new Pose with the same x/y as the robot,
     * but a heading that aims at the selected goal.
     */
    public static Pose computeAimPose(Pose currentPose, Goal goal) {
        double baseGoalX = (goal == Goal.BLUE_GOAL) ? BLUE_GOAL_X : RED_GOAL_X;

        double targetX = computeDynamicGoalX(currentPose.getX(), baseGoalX);
        double targetY = computeDynamicGoalY(currentPose.getY());

        double deltaX = targetX - currentPose.getX();
        double deltaY = targetY - currentPose.getY();

        double headingRad = Math.atan2(deltaY, deltaX);

        return new Pose(
                currentPose.getX(),
                currentPose.getY(),
                headingRad
        );
    }

    /**
     * Convenience if you only want the heading for a turn controller.
     */
    public static double computeAimHeadingRad(Pose currentPose, Goal goal) {
        return computeAimPose(currentPose, goal).getHeading();
    }

    // ------------------------------------------------------------
    // INTERNAL COMPUTATION
    // ------------------------------------------------------------

    private static double computeDynamicGoalX(double robotX, double baseGoalX) {
        double distanceFromGoalX = Math.abs(baseGoalX - robotX);

        double offsetFactor =
                clamp(FIELD_CENTER - distanceFromGoalX, 0.0, FIELD_CENTER)
                        / FIELD_CENTER;

        double inwardOffset = offsetFactor * OFFSET_MAX;

        return (baseGoalX == BLUE_GOAL_X)
                ? baseGoalX + inwardOffset
                : baseGoalX - inwardOffset;
    }

    private static double computeDynamicGoalY(double robotY) {
        double offsetFactor =
                clamp(robotY - FIELD_CENTER, 0.0, FIELD_CENTER)
                        / FIELD_CENTER;

        return lerp(GOAL_Y, GOAL_Y - OFFSET_MAX, offsetFactor);
    }

    // ------------------------------------------------------------
    // UTILS
    // ------------------------------------------------------------

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double lerp(double start, double end, double t) {
        return start + (end - start) * t;
    }
}
