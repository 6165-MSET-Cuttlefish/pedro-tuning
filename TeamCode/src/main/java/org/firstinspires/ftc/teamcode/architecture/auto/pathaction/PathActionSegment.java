package org.firstinspires.ftc.teamcode.architecture.auto.pathaction;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.lib.Action;
import org.firstinspires.ftc.teamcode.lib.Module;
import org.firstinspires.ftc.teamcode.lib.State;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;

public class PathActionSegment {
    private final PathChain path;
    private final List<Callable<Boolean>> stopConditions;
    private final ConditionMode stopMode;
    private final List<Action> duringActions;
    private final List<Action> afterActions;
    private final List<Callable<Boolean>> continueConditions;
    private final ConditionMode continueMode;
    private final Integer timeoutMs;
    private final Map<Module, State> moduleStates;

    private PathActionSegment(Builder builder) {
        this.path = builder.path;
        this.stopConditions = Collections.unmodifiableList(new ArrayList<>(builder.stopConditions));
        this.stopMode = builder.stopMode;
        this.duringActions = Collections.unmodifiableList(new ArrayList<>(builder.duringActions));
        this.afterActions = Collections.unmodifiableList(new ArrayList<>(builder.afterActions));
        this.continueConditions =
                Collections.unmodifiableList(new ArrayList<>(builder.continueConditions));
        this.continueMode = builder.continueMode;
        this.timeoutMs = builder.timeoutMs;
        this.moduleStates = Collections.unmodifiableMap(new HashMap<>(builder.moduleStates));
    }

    public PathChain getPath() {
        return path;
    }

    public List<Callable<Boolean>> getStopConditions() {
        return stopConditions;
    }

    public ConditionMode getStopMode() {
        return stopMode;
    }

    public List<Action> getDuringActions() {
        return duringActions;
    }

    public List<Action> getAfterActions() {
        return afterActions;
    }

    public List<Callable<Boolean>> getContinueConditions() {
        return continueConditions;
    }

    public ConditionMode getContinueMode() {
        return continueMode;
    }

    public Integer getTimeoutMs() {
        return timeoutMs;
    }

    public Map<Module, State> getModuleStates() {
        return moduleStates;
    }

    public boolean hasPath() {
        return path != null;
    }

    public boolean hasActions() {
        return !duringActions.isEmpty() || !afterActions.isEmpty();
    }

    public boolean isWait() {
        return !hasPath() && !hasActions() && !continueConditions.isEmpty();
    }

    public boolean isDelay() {
        return !hasPath() && !hasActions() && continueConditions.isEmpty() && timeoutMs != null
                && timeoutMs > -1;
    }

    public static PathActionSegment path(PathChain path) {
        return new Builder().path(path).build();
    }

    public static PathActionSegment action(Action... actions) {
        return new Builder().afterActions(Arrays.asList(actions)).build();
    }

    public static PathActionSegment delay(int delayMs) {
        return new Builder().timeoutMs(delayMs).build();
    }

    public static PathActionSegment waitFor(Callable<Boolean> condition, int timeoutMs) {
        return new Builder()
                .continueConditions(Collections.singletonList(condition))
                .continueMode(ConditionMode.OR)
                .timeoutMs(timeoutMs)
                .build();
    }

    public static class Builder {
        private PathChain path = null;
        private List<Callable<Boolean>> stopConditions = new ArrayList<>();
        private ConditionMode stopMode = ConditionMode.OR;
        private List<Action> duringActions = new ArrayList<>();
        private List<Action> afterActions = new ArrayList<>();
        private List<Callable<Boolean>> continueConditions = new ArrayList<>();
        private ConditionMode continueMode = ConditionMode.AND;
        private Integer timeoutMs = null;
        private Map<Module, State> moduleStates = new HashMap<>();

        public Builder path(PathChain path) {
            this.path = path;
            return this;
        }

        public Builder stopConditions(List<Callable<Boolean>> c) {
            this.stopConditions = c;
            return this;
        }

        public Builder stopMode(ConditionMode mode) {
            this.stopMode = mode;
            return this;
        }

        public Builder duringActions(List<Action> a) {
            this.duringActions = a;
            return this;
        }

        public Builder afterActions(List<Action> a) {
            this.afterActions = a;
            return this;
        }

        public Builder continueConditions(List<Callable<Boolean>> c) {
            this.continueConditions = c;
            return this;
        }

        public Builder continueMode(ConditionMode mode) {
            this.continueMode = mode;
            return this;
        }

        public Builder timeoutMs(Integer ms) {
            this.timeoutMs = ms;
            return this;
        }

        public Builder moduleStates(Map<Module, State> states) {
            this.moduleStates = states;
            return this;
        }

        public PathActionSegment build() {
            return new PathActionSegment(this);
        }
    }
}
