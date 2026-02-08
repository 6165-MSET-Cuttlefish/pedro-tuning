package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Base class for robot subsystem modules with state machine support.
 * Extend this class and implement initStates(), read(), and write().
 */
public abstract class Module {
    private static final int DEFAULT_HISTORY_SIZE = 50;
    private int maxHistorySize = DEFAULT_HISTORY_SIZE;
    
    private final List<State> states = new ArrayList<>();
    private final List<TransitionGuard<?>> guards = new ArrayList<>();
    private final List<StateHook> enterHooks = new ArrayList<>();
    private final List<StateHook> exitHooks = new ArrayList<>();
    private final List<State> stateHistory = new ArrayList<>();

    private final ElapsedTime stateTimer = new ElapsedTime();
    private Telemetry telemetry;
    private Action defaultAction;
    private boolean telemetryEnabled = true;
    private boolean writeEnabled = true;
    private String name;

    public Module() {
        this.name = getClass().getSimpleName();
        initStates();
    }

    /** Initialize states for this module. Call setStates() here. */
    protected abstract void initStates();

    /** Read sensor inputs. Called each loop before onLoop(). */
    protected abstract void read();

    /** Write command outputs to hardware. Called each loop after onLoop(). */
    protected abstract void write();

    /** Called once after module discovery. Override for additional setup. */
    public void init() {}

    /** Called after any state transition. Override to react to changes. */
    protected void onStateChange() {}

    /** Override to add custom telemetry for this module. */
    protected void onTelemetry() {}

    /**
     * Register initial states for this module. States of the same class
     * are mutually exclusive—only one can be active at a time.
     */
    protected final void setStates(State... initialStates) {
        states.clear();
        for (State s : initialStates) {
            s.setModule(this);
            states.add(s);

            if (s instanceof Enum<?>) {
                for (State variant : s.getClass().getEnumConstants()) {
                    variant.setModule(this);
                }
            }
        }
    }

    /**
     * Get the current state instance of the given type.
     * @throws IllegalArgumentException if no state of that type exists
     */
    @SuppressWarnings("unchecked")
    public final <T extends State> T get(Class<T> stateClass) {
        // Retrieve the current state instance of the given type
        for (State s : states) {
            if (stateClass.isInstance(s)) {
                return (T) s;
            }
        }
        throw new IllegalArgumentException("No state of type: " + stateClass.getSimpleName());
    }

    /**
     * Transition to a new state. Returns true if the transition succeeded,
     * false if blocked by guards or constraints.
     */
    public final boolean set(State newState) {
        for (int i = 0; i < states.size(); i++) {
            State current = states.get(i);
            if (newState.getClass() == current.getClass() && !newState.equals(current)) {
                // Check if transition is allowed by guards and constraints
                if (!checkGuards(current, newState)) {
                    return false;
                }

                if (!Constraints.checkAndHandle(newState)) {
                    return false;
                }

                // Fire exit hook for the old state before transition
                fireExitHooks(current);

                // Track state history for debugging
                stateHistory.add(current);
                if (stateHistory.size() > maxHistorySize) {
                    stateHistory.remove(0);
                    if (telemetryEnabled) {
                        System.err.println("[" + name + "] State history limit reached; oldest entry removed.");
                    }
                }

                states.set(i, newState);
                newState.setModule(this);
                stateTimer.reset();

                // Fire enter hook for the new state after transition
                fireEnterHooks(newState);

                onStateChange();
                return true;
            }
        }
        return false;
    }

    /**
     * Check if this module is currently in any of the given states.
     */
    public final boolean isIn(State... checkStates) {
        for (State check : checkStates) {
            State current = get(check.getClass());
            if (current.equals(check))
                return true;
        }
        return false;
    }

    /**
     * Get the time in milliseconds since the last state transition.
     */
    public final long stateTimeMs() {
        return (long) stateTimer.milliseconds();
    }

    /**
     * Get an unmodifiable list of previous states (most recent last).
     */
    public final List<State> getHistory() {
        return Collections.unmodifiableList(stateHistory);
    }

    /**
     * Set the maximum number of states to retain in history.
     */
    public final void setMaxHistorySize(int size) {
        this.maxHistorySize = Math.max(1, size);
    }

    /** Get the current maximum history size. */
    public final int getMaxHistorySize() {
        return maxHistorySize;
    }

    /**
     * Register a guard that can block transitions for the given state type.
     * @param stateClass The state type to guard
     * @param guard A function that returns false to block the transition
     */
    protected final <T extends State> void guard(Class<T> stateClass, TransitionCheck<T> guard) {
        guards.add(new TransitionGuard<>(stateClass, guard));
    }

    /**
     * Register a hook that fires when transitioning INTO the given state.
     */
    protected final void onEnter(State state, Runnable action) {
        enterHooks.add(new StateHook(state, action));
    }

    /**
     * Register a hook that fires when transitioning OUT OF the given state.
     */
    protected final void onExit(State state, Runnable action) {
        exitHooks.add(new StateHook(state, action));
    }

    @SuppressWarnings("unchecked")
    private <T extends State> boolean checkGuards(State from, State to) {
        for (TransitionGuard<?> g : guards) {
            if (g.stateClass == from.getClass()) {
                TransitionGuard<T> typedGuard = (TransitionGuard<T>) g;
                if (!typedGuard.check.test((T) from, (T) to)) {
                    return false;
                }
            }
        }
        return true;
    }

    private void fireEnterHooks(State state) {
        for (StateHook h : enterHooks) {
            if (h.state.equals(state)) {
                h.action.run();
            }
        }
    }

    private void fireExitHooks(State state) {
        for (StateHook h : exitHooks) {
            if (h.state.equals(state)) {
                h.action.run();
            }
        }
    }

    final void setTelemetry(Telemetry t) {
        this.telemetry = t;
    }

    protected final Telemetry getTelemetry() {
        return telemetry;
    }

    final void telemetry() {
        if (!telemetryEnabled || telemetry == null)
            return;

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < states.size(); i++) {
            if (i > 0)
                sb.append(" | ");
            sb.append(states.get(i));
        }
        telemetry.addData(name, sb.toString());
        onTelemetry();
    }

    /**
     * Set a display name for this module (used in telemetry and error messages).
     * @return this module for chaining
     */
    public final Module named(String name) {
        this.name = name;
        return this;
    }

    /** Get the display name of this module. */
    public final String getName() {
        return name;
    }

    /** Enable or disable automatic telemetry output for this module. */
    public final void setTelemetryEnabled(boolean enabled) {
        this.telemetryEnabled = enabled;
    }

    /** Enable or disable hardware writes during OpMode.loop(). */
    public final void setWriteEnabled(boolean enabled) {
        this.writeEnabled = enabled;
    }

    /** Check if hardware writes are enabled for this module. */
    public final boolean isWriteEnabled() {
        return writeEnabled;
    }

    /**
     * Set a default Action to run automatically when the OpMode starts.
     */
    public final void setDefaultAction(Action action) {
        this.defaultAction = action;
    }

    /** Get the default action, or null if none is set. */
    public final Action getDefaultAction() {
        return defaultAction;
    }

    /**
     * Log a value to telemetry with the module name prefix.
     */
    protected final void log(String caption, Object value) {
        if (telemetryEnabled && telemetry != null) {
            telemetry.addData(name + " " + caption, value);
        }
    }

    /**
     * Log a formatted string to telemetry with the module name prefix.
     */
    protected final void log(String caption, String format, Object... args) {
        if (telemetryEnabled && telemetry != null) {
            telemetry.addData(name + " " + caption, String.format(format, args));
        }
    }

    /**
     * Functional interface for transition guards.
     * Return false to block a state transition.
     */
    @FunctionalInterface
    public interface TransitionCheck<T extends State> {
        boolean test(T from, T to);
    }

    private static class TransitionGuard<T extends State> {
        final Class<T> stateClass;
        final TransitionCheck<T> check;

        TransitionGuard(Class<T> stateClass, TransitionCheck<T> check) {
            this.stateClass = stateClass;
            this.check = check;
        }
    }

    private static class StateHook {
        final State state;
        final Runnable action;

        StateHook(State state, Runnable action) {
            this.state = state;
            this.action = action;
        }
    }
}
