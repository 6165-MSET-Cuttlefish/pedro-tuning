package org.firstinspires.ftc.teamcode.architecture.auto.pathaction;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.*;
import com.pedropathing.paths.callbacks.PathCallback;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.lib.Action;
import org.firstinspires.ftc.teamcode.lib.ActionBuilder;
import org.firstinspires.ftc.teamcode.lib.Actions;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;

public class IntegratedPathBuilder {
    private final Pose startPose;
    private final PathBuilder pedroBuilder;
    private Pose lastPose;
    private final List<Action> duringActions = new ArrayList<>();

    IntegratedPathBuilder(Pose startPose) {
        this.startPose = startPose;
        this.pedroBuilder = Robot.robot.follower.pathBuilder();
        this.lastPose = startPose;
    }

    public Pose getStartPose() {
        return startPose;
    }

    public IntegratedPathBuilder addLine(Pose endPose) {
        pedroBuilder.addPath(new BezierLine(lastPose, endPose));
        lastPose = endPose;
        return this;
    }

    public IntegratedPathBuilder addCurve(Pose controlPoint, Pose endPose) {
        pedroBuilder.addPath(new BezierCurve(lastPose, controlPoint, endPose));
        lastPose = endPose;
        return this;
    }

    public IntegratedPathBuilder addCurve(Pose control1, Pose control2, Pose endPose) {
        pedroBuilder.addPath(new BezierCurve(lastPose, control1, control2, endPose));
        lastPose = endPose;
        return this;
    }

    public IntegratedPathBuilder addCurve(Pose control1, Pose control2, Pose control3, Pose endPose) {
        pedroBuilder.addPath(new BezierCurve(lastPose, control1, control2, control3, endPose));
        lastPose = endPose;
        return this;
    }

    public IntegratedPathBuilder addPath(Path path) {
        pedroBuilder.addPath(path);
        Pose endPose = path.endPose();
        lastPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());
        return this;
    }

    public IntegratedPathBuilder addPath(Curve curve) {
        pedroBuilder.addPath(curve);
        lastPose = curve.getLastControlPoint();
        return this;
    }

    public IntegratedPathBuilder addPaths(Path... paths) {
        pedroBuilder.addPaths(paths);
        if (paths.length > 0) {
            Pose endPose = paths[paths.length - 1].endPose();
            lastPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());
        }
        return this;
    }

    public IntegratedPathBuilder addPaths(Curve... curves) {
        pedroBuilder.addPaths(curves);
        if (curves.length > 0) {
            lastPose = curves[curves.length - 1].getLastControlPoint();
        }
        return this;
    }

    public IntegratedPathBuilder curveThrough(Pose prevPoint, Pose startPoint, double tension, Pose... points) {
        pedroBuilder.curveThrough(prevPoint, startPoint, tension, points);
        if (points.length > 0) {
            lastPose = points[points.length - 1];
        } else {
            lastPose = startPoint;
        }
        return this;
    }

    public IntegratedPathBuilder curveThrough(double tension, Pose... points) {
        pedroBuilder.curveThrough(tension, points);
        if (points.length > 0) {
            lastPose = points[points.length - 1];
        }
        return this;
    }

    public IntegratedPathBuilder setConstantHeading(double heading) {
        pedroBuilder.setConstantHeadingInterpolation(heading);
        return this;
    }

    public IntegratedPathBuilder setConstantHeadingInterpolation(double heading) {
        return setConstantHeading(heading);
    }

    public IntegratedPathBuilder setGlobalConstantHeading(double heading) {
        pedroBuilder.setGlobalConstantHeadingInterpolation(heading);
        return this;
    }

    public IntegratedPathBuilder setGlobalConstantHeadingInterpolation(double heading) {
        return setGlobalConstantHeading(heading);
    }

    public IntegratedPathBuilder setLinearHeading(double startHeading, double endHeading) {
        pedroBuilder.setLinearHeadingInterpolation(startHeading, endHeading);
        return this;
    }

    public IntegratedPathBuilder setLinearHeadingInterpolation(double startHeading, double endHeading) {
        return setLinearHeading(startHeading, endHeading);
    }

    public IntegratedPathBuilder setLinearHeading(double startHeading, double endHeading, double endTime) {
        pedroBuilder.setLinearHeadingInterpolation(startHeading, endHeading, endTime);
        return this;
    }

    public IntegratedPathBuilder setLinearHeadingInterpolation(double startHeading, double endHeading, double endTime) {
        return setLinearHeading(startHeading, endHeading, endTime);
    }

    public IntegratedPathBuilder setLinearHeading(
            double startHeading, double endHeading, double startTime, double endTime) {
        pedroBuilder.setLinearHeadingInterpolation(startHeading, endHeading, endTime, startTime);
        return this;
    }

    public IntegratedPathBuilder setLinearHeadingInterpolation(
            double startHeading, double endHeading, double startTime, double endTime) {
        return setLinearHeading(startHeading, endHeading, startTime, endTime);
    }

    public IntegratedPathBuilder setGlobalLinearHeading(double startHeading, double endHeading) {
        pedroBuilder.setGlobalLinearHeadingInterpolation(startHeading, endHeading);
        return this;
    }

    public IntegratedPathBuilder setGlobalLinearHeadingInterpolation(double startHeading, double endHeading) {
        return setGlobalLinearHeading(startHeading, endHeading);
    }

    public IntegratedPathBuilder setGlobalLinearHeading(double startHeading, double endHeading, double endTime) {
        pedroBuilder.setGlobalLinearHeadingInterpolation(startHeading, endHeading, endTime);
        return this;
    }

    public IntegratedPathBuilder setGlobalLinearHeadingInterpolation(double startHeading, double endHeading, double endTime) {
        return setGlobalLinearHeading(startHeading, endHeading, endTime);
    }

    public IntegratedPathBuilder setGlobalLinearHeading(double startHeading, double endHeading, double startTime, double endTime) {
        pedroBuilder.setGlobalLinearHeadingInterpolation(startHeading, endHeading, endTime, startTime);
        return this;
    }

    public IntegratedPathBuilder setGlobalLinearHeadingInterpolation(double startHeading, double endHeading, double startTime, double endTime) {
        return setGlobalLinearHeading(startHeading, endHeading, startTime, endTime);
    }

    public IntegratedPathBuilder setTangentHeading() {
        pedroBuilder.setTangentHeadingInterpolation();
        return this;
    }

    public IntegratedPathBuilder setTangentHeadingInterpolation() {
        return setTangentHeading();
    }

    public IntegratedPathBuilder setGlobalTangentHeading() {
        pedroBuilder.setGlobalTangentHeadingInterpolation();
        return this;
    }

    public IntegratedPathBuilder setGlobalTangentHeadingInterpolation() {
        return setGlobalTangentHeading();
    }

    public IntegratedPathBuilder setReversed() {
        pedroBuilder.setReversed();
        return this;
    }

    public IntegratedPathBuilder setGlobalReversed() {
        pedroBuilder.setGlobalReversed();
        return this;
    }

    public IntegratedPathBuilder setHeadingInterpolation(HeadingInterpolator function) {
        pedroBuilder.setHeadingInterpolation(function);
        return this;
    }

    public IntegratedPathBuilder setGlobalHeadingInterpolation(HeadingInterpolator function) {
        pedroBuilder.setGlobalHeadingInterpolation(function);
        return this;
    }

    public IntegratedPathBuilder setBrakingStrength(int strength) {
        pedroBuilder.setBrakingStrength(strength);
        return this;
    }

    public IntegratedPathBuilder setBrakingStrength(double strength) {
        pedroBuilder.setBrakingStrength(strength);
        return this;
    }

    public IntegratedPathBuilder setBrakingStart(double set) {
        pedroBuilder.setBrakingStart(set);
        return this;
    }

    public IntegratedPathBuilder setVelocityConstraint(double set) {
        pedroBuilder.setVelocityConstraint(set);
        return this;
    }

    public IntegratedPathBuilder setTranslationalConstraint(double set) {
        pedroBuilder.setTranslationalConstraint(set);
        return this;
    }

    public IntegratedPathBuilder setHeadingConstraint(double set) {
        pedroBuilder.setHeadingConstraint(set);
        return this;
    }

    public IntegratedPathBuilder setTValueConstraint(double set) {
        pedroBuilder.setTValueConstraint(set);
        return this;
    }

    public IntegratedPathBuilder setTimeoutConstraint(double set) {
        pedroBuilder.setTimeoutConstraint(set);
        return this;
    }

    public IntegratedPathBuilder addTemporalCallback(double time, Runnable runnable) {
        pedroBuilder.addTemporalCallback(time, runnable);
        return this;
    }

    public IntegratedPathBuilder addParametricCallback(double t, Runnable runnable) {
        pedroBuilder.addParametricCallback(t, runnable);
        return this;
    }

    public IntegratedPathBuilder addPoseCallback(Pose targetPoint, Runnable runnable, double initialTValueGuess) {
        pedroBuilder.addPoseCallback(targetPoint, runnable, initialTValueGuess);
        return this;
    }

    public IntegratedPathBuilder addCallback(PathCallback callback) {
        pedroBuilder.addCallback(callback);
        return this;
    }

    public IntegratedPathBuilder addCallback(PathCallback callback, int i) {
        pedroBuilder.addCallback(callback, i);
        return this;
    }

    public IntegratedPathBuilder addCallback(PathBuilder.CallbackCondition condition, Runnable action) {
        pedroBuilder.addCallback(condition, action);
        return this;
    }

    public IntegratedPathBuilder addCallback(PathBuilder.CallbackCondition condition, Runnable action, int i) {
        pedroBuilder.addCallback(condition, action, i);
        return this;
    }

    public IntegratedPathBuilder addLoopedCallback(PathCallback callback) {
        pedroBuilder.addLoopedCallback(callback);
        return this;
    }

    public IntegratedPathBuilder setGlobalDeceleration() {
        pedroBuilder.setGlobalDeceleration();
        return this;
    }

    public IntegratedPathBuilder setGlobalDeceleration(double brakingStart) {
        pedroBuilder.setGlobalDeceleration(brakingStart);
        return this;
    }

    public IntegratedPathBuilder setNoDeceleration() {
        pedroBuilder.setNoDeceleration();
        return this;
    }

    public IntegratedPathBuilder setConstraints(PathConstraints constraints) {
        pedroBuilder.setConstraints(constraints);
        return this;
    }

    public IntegratedPathBuilder setConstraintsForAll(PathConstraints constraints) {
        pedroBuilder.setConstraintsForAll(constraints);
        return this;
    }

    public IntegratedPathBuilder setConstraintsForLast(PathConstraints constraints) {
        pedroBuilder.setConstraintsForLast(constraints);
        return this;
    }

    public IntegratedPathBuilder during(Action action) {
        duringActions.add(action);
        return this;
    }

    public IntegratedPathBuilder whenDuring(Callable<Boolean> condition, Runnable code) {
        ActionBuilder builder = Actions.builder();
        builder.waitUntil(() -> {
            try {
                return condition.call();
            } catch (Exception e) {
                return false;
            }
        });
        builder.run(code);
        duringActions.add(builder.build());
        return this;
    }

    public IntegratedPathBuilder whenDuring(Callable<Boolean> condition, Action action) {
        ActionBuilder builder = Actions.builder();
        builder.waitUntil(() -> {
            try {
                return condition.call();
            } catch (Exception e) {
                return false;
            }
        });
        builder.action(action);
        duringActions.add(builder.build());
        return this;
    }

    PathChain buildPath() {
        return pedroBuilder.build();
    }

    List<Action> getDuringActions() {
        return new ArrayList<>(duringActions);
    }
}
