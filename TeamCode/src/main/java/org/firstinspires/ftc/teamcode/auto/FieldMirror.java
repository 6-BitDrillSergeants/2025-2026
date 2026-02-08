package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.config.Goal;

public final class FieldMirror {

    public static final double FIELD_SIZE = 144.0;

    private FieldMirror() {}

    public static Pose getPose(Pose bluePose, Goal desiredGoal) {
        if (desiredGoal == Goal.BLUE) return bluePose;

        return new Pose(
                FIELD_SIZE - bluePose.getX(),
                bluePose.getY(),
                getHeading(bluePose.getHeading(), desiredGoal)
        );
    }

    /**
     * Returns the correct target heading for the specified goal color.
     *
     * We tune all shot headings assuming the BLUE goal coordinate frame.
     * When aiming at the RED goal, we mirror that heading across the field's
     * vertical centerline (x-axis flip), which geometrically becomes:
     *
     *      mirrored = π - blueHeading
     *
     * The result is then normalized to the range [-π, π] so downstream
     * controllers never see discontinuities or wraparound jumps.
     */
    public static double getHeading(double blueHeading, Goal goal) {
        if (goal == Goal.BLUE) {
            return blueHeading;
        }

        // Mirror across field centerline and normalize to [-π, π]
        double mirrored = Math.PI - blueHeading;
        return Math.atan2(Math.sin(mirrored), Math.cos(mirrored));
    }
}