package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Goal;

import java.util.ArrayList;
import java.util.List;

public final class AutoPathSpec {

    public interface SegmentSpec {
        void applyTo(PathBuilder builder, Goal goal);
    }

    public interface HeadingSpec {
        void applyTo(PathBuilder builder, Goal goal);
    }

    private final List<SegmentSpec> segments = new ArrayList<>();
    private HeadingSpec headingSpec = null;

    public AutoPathSpec addLine(Pose start, Pose end) {
        segments.add((builder, goal) -> builder.addPath(
                new com.pedropathing.geometry.BezierLine(
                        FieldMirror.getPose(start, goal),
                        FieldMirror.getPose(end, goal)
                )
        ));
        return this;
    }

    public AutoPathSpec addCurve(Pose startPose, Pose controlPoint, Pose endPoint) {
        segments.add((builder, goal) -> builder.addPath(
                new com.pedropathing.geometry.BezierCurve(
                        FieldMirror.getPose(startPose, goal),
                        FieldMirror.getPose(controlPoint, goal),
                        FieldMirror.getPose(endPoint, goal)
                )
        ));
        return this;
    }

    public AutoPathSpec constantHeading(double headingRad) {
        headingSpec = (builder, goal) ->
                builder.setConstantHeadingInterpolation(FieldMirror.getHeading(headingRad, goal));
        return this;
    }

    public AutoPathSpec tangentHeading() {
        headingSpec = (builder, goal) -> builder.setTangentHeadingInterpolation();
        return this;
    }

    public AutoPathSpec linearHeading(double startHeadingRad, double endHeadingRad) {
        headingSpec = (builder, goal) ->
                builder.setLinearHeadingInterpolation(
                        FieldMirror.getHeading(startHeadingRad, goal),
                        FieldMirror.getHeading(endHeadingRad, goal)
                );
        return this;
    }

    public PathChain build(Follower follower, Goal goal) {
        PathBuilder builder = follower.pathBuilder();
        for (SegmentSpec segment : segments) {
            segment.applyTo(builder, goal);
        }
        if (headingSpec != null) {
            headingSpec.applyTo(builder, goal);
        }
        return builder.build();
    }
}
