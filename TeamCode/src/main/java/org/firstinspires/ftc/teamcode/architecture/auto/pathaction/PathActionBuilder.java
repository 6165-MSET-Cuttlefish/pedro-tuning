package org.firstinspires.ftc.teamcode.architecture.auto.pathaction;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.lib.Action;
import org.firstinspires.ftc.teamcode.lib.Actions;
import org.firstinspires.ftc.teamcode.lib.Module;
import org.firstinspires.ftc.teamcode.lib.State;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.Callable;
import java.util.function.Consumer;

public class PathActionBuilder {
    private final List<PathActionSegment> segments = new ArrayList<>();
    private Pose lastPathEndPose = null;
    private final Map<Module, State> moduleStates = new HashMap<>();

    private void clearModuleStates() {
        moduleStates.clear();
    }

    private PathActionSegment createPathSegment(PathChain path,
            List<Callable<Boolean>> stopConditions, ConditionMode stopMode, List<Action> actions,
            boolean blocking, List<Action> duringPathActions) {
        Objects.requireNonNull(
                path, "PathActionBuilder.createPathSegment requires a non-null path");

        Pose endPose = path.endPose();
        lastPathEndPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());

        PathActionSegment.Builder builder = new PathActionSegment.Builder()
                                                    .path(path)
                                                    .stopConditions(stopConditions)
                                                    .stopMode(stopMode);

        if (blocking) {
            builder.duringActions(duringPathActions).afterActions(actions);
        } else {
            builder.duringActions(actions);
        }

        builder.moduleStates(new HashMap<>(moduleStates));

        clearModuleStates();
        return builder.build();
    }

    private PathActionSegment createActionSegment(
            List<Action> actions, boolean blocking, List<Action> duringPathActions) {
        PathActionSegment.Builder builder = new PathActionSegment.Builder()
                                                    .stopConditions(new ArrayList<>())
                                                    .stopMode(ConditionMode.OR);

        if (blocking) {
            builder.duringActions(duringPathActions).afterActions(actions);
        } else {
            builder.duringActions(actions);
        }

        builder.moduleStates(new HashMap<>(moduleStates));

        clearModuleStates();
        return builder.build();
    }

    @SafeVarargs
    public final PathActionBuilder path(PathChain path, Callable<Boolean>... stopWhen) {
        segments.add(createPathSegment(path, Arrays.asList(stopWhen), ConditionMode.OR,
                new ArrayList<>(), true, new ArrayList<>()));
        return this;
    }

    public PathActionBuilder setStartPose(Pose startPose) {
        lastPathEndPose = startPose;
        return this;
    }

    public PathActionBuilder buildPath(Consumer<IntegratedPathBuilder> builder) {
        return buildPath(lastPathEndPose, builder);
    }

    public PathActionBuilder buildPath(Pose startPose, Consumer<IntegratedPathBuilder> builder) {
        Pose safeStart = Objects.requireNonNull(
                startPose, "startPose is required; call setStartPose() before buildPath");
        IntegratedPathBuilder pathBuilder = new IntegratedPathBuilder(safeStart);
        builder.accept(pathBuilder);
        PathChain path = pathBuilder.buildPath();
        List<Action> during = pathBuilder.getDuringActions();
        segments.add(createPathSegment(
                path, new ArrayList<>(), ConditionMode.OR, new ArrayList<>(), true, during));
        return this;
    }

    public PathActionBuilder action(Action action) {
        segments.add(
                createActionSegment(Collections.singletonList(action), true, new ArrayList<>()));
        return this;
    }

    public PathActionBuilder actionAsync(Action action) {
        segments.add(
                createActionSegment(Collections.singletonList(action), false, new ArrayList<>()));
        return this;
    }

    public PathActionBuilder actions(Action... actions) {
        segments.add(createActionSegment(Arrays.asList(actions), true, new ArrayList<>()));
        return this;
    }

    public PathActionBuilder actionsAsync(Action... actions) {
        segments.add(createActionSegment(Arrays.asList(actions), false, new ArrayList<>()));
        return this;
    }

    public PathActionBuilder setState(State... states) {
        for (State state : states) {
            moduleStates.put(state.getModule(), state);
        }
        return this;
    }

    public PathActionBuilder await(Callable<Boolean> condition, int timeoutMs) {
        PathActionSegment segment =
                new PathActionSegment.Builder()
                        .continueConditions(Collections.singletonList(condition))
                        .continueMode(ConditionMode.OR)
                        .timeoutMs(timeoutMs)
                        .build();
        segments.add(segment);
        return this;
    }

    public PathActionBuilder await(Callable<Boolean> condition) {
        return await(condition, 5000);
    }

    @SafeVarargs
    public final PathActionBuilder awaitAll(int timeoutMs, Callable<Boolean>... conditions) {
        PathActionSegment segment = new PathActionSegment.Builder()
                                            .continueConditions(Arrays.asList(conditions))
                                            .continueMode(ConditionMode.AND)
                                            .timeoutMs(timeoutMs)
                                            .build();
        segments.add(segment);
        return this;
    }

    public PathActionBuilder delay(int delayMs) {
        PathActionSegment segment = new PathActionSegment.Builder()
                .moduleStates(new HashMap<>(moduleStates))
                .timeoutMs(delayMs)
                .build();
        segments.add(segment);
        clearModuleStates();
        return this;
    }

    public PathActionBuilder run(Runnable code) {
        return action(Actions.builder().run(code).build());
    }

    private Callable<Boolean> overrideCondition = null;
    private Runnable overrideHandler = null;

    public PathActionBuilder setOverride(Callable<Boolean> condition, Runnable handler) {
        this.overrideCondition = condition;
        this.overrideHandler = handler;
        return this;
    }

    public PathActionBuilder setTimeOverride(int timeMs, Runnable handler) {
        this.overrideCondition = () -> Robot.robot.opMode.getGameTimer().milliseconds() >= timeMs;
        this.overrideHandler = handler;
        return this;
    }

    public PathActionScheduler build() {
        PathActionScheduler scheduler = new PathActionScheduler();
        segments.forEach(scheduler::addSegment);

        scheduler.setOverride(overrideCondition, overrideHandler);

        return scheduler;
    }
}
