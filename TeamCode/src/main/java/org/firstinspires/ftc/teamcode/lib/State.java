package org.firstinspires.ftc.teamcode.lib;

import java.util.concurrent.ConcurrentHashMap;

public interface State {
    ConcurrentHashMap<State, Double> VALUES = new ConcurrentHashMap<>();
    ConcurrentHashMap<State, Module> MODULES = new ConcurrentHashMap<>();

    /**
     * Get the cached numeric value associated with this state.
     * Used for states that need to track a scalar value (e.g., servo positions, motor speeds).
     */
    default double getValue() {
        return VALUES.getOrDefault(this, 0.0);
    }

    /**
     * Set a numeric value for this state.
     */
    default void setValue(double value) {
        VALUES.put(this, value);
    }

    /**
     * Get the Module that owns this state.
     */
    default Module getModule() {
        return MODULES.get(this);
    }

    /**
     * Associate this state with a module (called automatically by Module.set()).
     */
    default void setModule(Module module) {
        MODULES.put(this, module);
    }

    /**
     * Transition to this state via its module.
     * Returns true if the transition succeeded, false if blocked by guards or constraints.
     */
    default boolean apply() {
        Module m = getModule();
        return m != null && m.set(this);
    }

    /**
     * Convert this state to an Action that sets it.
     */
    default Action asAction() {
        return Actions.set(this);
    }
}
