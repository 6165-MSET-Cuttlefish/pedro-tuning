package org.firstinspires.ftc.teamcode.architecture.auto.pathaction;

import android.os.SystemClock;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.lib.Action;
import org.firstinspires.ftc.teamcode.lib.Actions;
import org.firstinspires.ftc.teamcode.lib.Module;
import org.firstinspires.ftc.teamcode.lib.State;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class PathActionScheduler {
    private final List<PathActionSegment> segments = new ArrayList<>();
    private int currentIndex = 0;
    private SchedulerState currentState = SchedulerState.IDLE;
    private long segmentStartTime = 0L;
    private Future<?> actionJob = null;
    private ExecutorService executorService = Executors.newSingleThreadExecutor();

    private final List<Callable<Boolean>> overrideConditions = new ArrayList<>();
    private Runnable overrideCallback = null;
    private boolean overrideTriggered = false;

    public PathActionScheduler addSegment(PathActionSegment segment) {
        segments.add(segment);
        return this;
    }

    public PathActionScheduler addSegments(PathActionSegment... segs) {
        Collections.addAll(segments, segs);
        return this;
    }

    public PathActionSegment getCurrentSegment() {
        if (currentIndex >= segments.size()) {
            return null;
        }
        return segments.get(currentIndex);
    }

    public PathChain getCurrentPath() {
        PathActionSegment segment = getCurrentSegment();
        if (segment == null) {
            return null;
        }
        return segment.getPath();
    }

    public SchedulerState getCurrentState() {
        return currentState;
    }

    public int getCurrentIndex() {
        return currentIndex;
    }

    public int getTotalSegments() {
        return segments.size();
    }

    public boolean isComplete() {
        return currentIndex >= segments.size();
    }

    public void reset() {
        currentIndex = 0;
        currentState = SchedulerState.IDLE;
        segmentStartTime = 0L;
        cancelAsyncOperations();
    }

    public void cancelAll() {
        cancelAsyncOperations();
        stopCurrentPath();
        Actions.cancelAll();
    }

    public void setOverride(Callable<Boolean> condition, Runnable callback) {
        overrideConditions.clear();
        overrideConditions.add(condition);
        this.overrideCallback = callback;
        this.overrideTriggered = false;
    }

    @SafeVarargs
    public final void setOverride(Runnable callback, Callable<Boolean>... conditions) {
        overrideConditions.clear();
        Collections.addAll(overrideConditions, conditions);
        this.overrideCallback = callback;
        this.overrideTriggered = false;
    }

    public boolean isOverrideTriggered() {
        return overrideTriggered;
    }

    public void triggerOverride() {
        if (!overrideTriggered) {
            overrideTriggered = true;
            cancelAll();
            overrideCallback.run();
        }
    }

    public void update() {
        if (!overrideTriggered && shouldTriggerOverride()) {
            triggerOverride();
            return;
        }

        if (overrideTriggered || isComplete())
            return;

        PathActionSegment segment = getCurrentSegment();
        if (currentState != SchedulerState.IDLE && hasTimedOut(segment)) {
            advanceToNextSegment();
            return;
        }

        switch (currentState) {
            case IDLE:
                initializeSegment(segment);
                break;
            case PATH_RUNNING:
                if (shouldStopPath(segment)) {
                    stopCurrentPath();
                    executeAfterActions(segment);
                } else if (!isPathBusy()) {
                    executeAfterActions(segment);
                }
                break;
            case AFTER_ACTION:
                if (areActionsComplete())
                    beginWaiting(segment);
                break;
            case WAITING:
                if (canContinue(segment))
                    advanceToNextSegment();
                break;
            case COMPLETED:
                advanceToNextSegment();
                break;
        }
    }

    public String getDebugInfo() {
        PathActionSegment seg = getCurrentSegment();
        StringBuilder sb = new StringBuilder();
        sb.append(currentIndex + 1).append("/").append(segments.size()).append(" | ");
        sb.append(currentState).append(" | ");

        long elapsed = segmentStartTime == 0 ? 0 : SystemClock.elapsedRealtime() - segmentStartTime;
        Integer timeout = seg.getTimeoutMs();
        sb.append(elapsed).append("/").append(timeout == null ? "inf" : timeout).append("ms | ");

        sb.append("P:").append(seg.hasPath());
        sb.append(" D:").append(seg.getDuringActions().size());
        sb.append(" A:").append(seg.getAfterActions().size());

        return sb.toString();
    }

    private void initializeSegment(PathActionSegment segment) {
        segmentStartTime = SystemClock.elapsedRealtime();
        cancelAsyncOperations();

        Map<Module, State> moduleStates = segment.getModuleStates();
        moduleStates.forEach(Module::set);

        beginPathFollowing(segment);
    }

    private void beginPathFollowing(PathActionSegment segment) {
        if (segment.hasPath()) {
            currentState = SchedulerState.PATH_RUNNING;
            Robot.robot.follower.followPath(segment.getPath(), true);
            segment.getDuringActions().forEach(Action::run);
        } else if (!segment.getDuringActions().isEmpty()) {
            segment.getDuringActions().forEach(Action::run);
            executeAfterActions(segment);
        } else {
            executeAfterActions(segment);
        }
    }

    private void executeAfterActions(PathActionSegment segment) {
        if (!segment.getAfterActions().isEmpty()) {
            currentState = SchedulerState.AFTER_ACTION;
            segment.getAfterActions().forEach(Action::run);
            actionJob = waitForActions(segment.getAfterActions());
        } else {
            beginWaiting(segment);
        }
    }

    private void beginWaiting(PathActionSegment segment) {
        currentState = (!segment.getContinueConditions().isEmpty() || segment.isDelay())
                ? SchedulerState.WAITING
                : SchedulerState.COMPLETED;
    }

    private void advanceToNextSegment() {
        cancelAsyncOperations();
        currentIndex++;
        currentState = SchedulerState.IDLE;
        segmentStartTime = 0L;
    }

    private Future<?> waitForActions(List<Action> actions) {
        return executorService.submit(() -> {
            try {
                while (actions.stream().anyMatch(Action::isRunning)
                        && Robot.robot.opMode.isRunning()) {
                    Thread.sleep(50);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        });
    }

    private void cancelAsyncOperations() {
        if (actionJob != null) {
            actionJob.cancel(true);
            actionJob = null;
        }
        if (executorService != null && !executorService.isShutdown()) {
            executorService.shutdownNow();
        }
        executorService = Executors.newSingleThreadExecutor();
    }

    private boolean areActionsComplete() {
        return actionJob == null || actionJob.isDone();
    }

    private boolean isPathBusy() {
        return Robot.robot.follower.isBusy();
    }

    private boolean hasTimedOut(PathActionSegment segment) {
        Integer timeout = segment.getTimeoutMs();
        if (timeout == null)
            return false;
        return SystemClock.elapsedRealtime() - segmentStartTime > timeout;
    }

    private boolean canContinue(PathActionSegment segment) {
        List<Callable<Boolean>> conditions = segment.getContinueConditions();
        if (conditions.isEmpty())
            return !segment.isDelay();

        boolean result = segment.getContinueMode() == ConditionMode.AND;
        for (Callable<Boolean> condition : conditions) {
            boolean eval = evaluateCondition(condition);
            if (segment.getContinueMode() == ConditionMode.AND) {
                result &= eval;
            } else {
                result |= eval;
            }
        }
        return result;
    }

    private boolean shouldStopPath(PathActionSegment segment) {
        List<Callable<Boolean>> conditions = segment.getStopConditions();
        if (conditions.isEmpty())
            return false;

        boolean result = segment.getStopMode() == ConditionMode.AND;
        for (Callable<Boolean> condition : conditions) {
            boolean eval = evaluateCondition(condition);
            if (segment.getStopMode() == ConditionMode.AND) {
                result &= eval;
            } else {
                result |= eval;
            }
        }
        return result;
    }

    private boolean evaluateCondition(Callable<Boolean> condition) {
        try {
            return condition.call();
        } catch (Exception e) {
            return false;
        }
    }

    private boolean shouldTriggerOverride() {
        return !overrideConditions.isEmpty()
                && overrideConditions.stream().anyMatch(this::evaluateCondition);
    }

    private void stopCurrentPath() {
        Robot.robot.follower.startTeleopDrive(true);
        Robot.robot.follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
    }

    public void skipCurrentSegment() {
        stopCurrentPath();
        advanceToNextSegment();
    }

    public void shutdown() {
        cancelAsyncOperations();
        executorService.shutdown();
    }
}
