package org.firstinspires.ftc.teamcode.lib;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

/**
 * Constraint system to prevent invalid state transitions.
 * Supports requirements (conditions that must be true), forbidden states,
 * and mutual exclusion groups.
 */
public final class Constraints {
    private static final List<Requirement> requirements = new ArrayList<>();
    private static final List<Forbidden> forbiddens = new ArrayList<>();
    private static final List<MutexGroup> mutexGroups = new ArrayList<>();
    private static ViolationHandler violationHandler = ViolationHandler.BLOCK;

    private Constraints() {}

    /**
     * Define constraints for the OpMode using a builder.
     * Example: Constraints.define(b -&gt; b.mutex(StateA, StateB));
     */
    public static void define(ConstraintDefiner definer) {
        definer.define(new ConstraintBuilder());
    }

    /**
     * Clear all registered constraints.
     * Called automatically at OpMode.stop().
     */
    public static void clear() {
        requirements.clear();
        forbiddens.clear();
        mutexGroups.clear();
    }

    /**
     * Set how constraint violations are handled.
     * @param handler One of BLOCK, LOG, or THROW
     */
    public static void onViolation(ViolationHandler handler) {
        violationHandler = handler;
    }

    /**
     * Check if a state transition is allowed without executing handlers.
     * @return ConstraintResult.ALLOWED or ConstraintResult.Blocked with reason
     */
    public static ConstraintResult check(State state) {
        // Check requirement constraints
        for (Requirement req : requirements) {
            if (req.targetState == state && !req.condition.getAsBoolean()) {
                return new ConstraintResult.Blocked(
                    "Requirement not met for " + state.getClass().getSimpleName()
                );
            }
        }

        // Check forbidden state constraints
        for (Forbidden forb : forbiddens) {
            if (forb.stateClass.isInstance(state) && forb.condition.getAsBoolean()) {
                return new ConstraintResult.Blocked(
                    "State " + state.getClass().getSimpleName() + " is forbidden"
                );
            }
        }

        // Check mutual exclusion constraints
        for (MutexGroup mutex : mutexGroups) {
            if (mutex.states.contains(state)) {
                List<State> conflicting = mutex.states.stream()
                    .filter(s -> s != state && isActive(s))
                    .collect(Collectors.toList());
                if (!conflicting.isEmpty()) {
                    StringBuilder details = new StringBuilder();
                    for (State conflict : conflicting) {
                        if (details.length() > 0) details.append(", ");
                        Module mod = conflict.getModule();
                        details.append(conflict.getClass().getSimpleName())
                               .append(" (in ").append(mod != null ? mod.getName() : "unknown").append(")");
                    }
                    return new ConstraintResult.Blocked(
                        "Mutex conflict: " + state.getClass().getSimpleName() + 
                        " (in " + (state.getModule() != null ? state.getModule().getName() : "unknown") + 
                        ") conflicts with [" + details + "]"
                    );
                }
            }
        }

        return ConstraintResult.ALLOWED;
    }

    /**
     * Check if a state is allowed without side effects.
     */
    public static boolean isAllowed(State state) {
        return check(state) == ConstraintResult.ALLOWED;
    }

    /**
     * Check constraints and execute the violation handler if violated.
     * @return true if allowed, false if blocked
     */
    public static boolean checkAndHandle(State state) {
        ConstraintResult result = check(state);
        if (result == ConstraintResult.ALLOWED) {
            return true;
        } else {
            violationHandler.handle(((ConstraintResult.Blocked) result).reason, state);
            return false;
        }
    }

    private static boolean isActive(State state) {
        Module module = state.getModule();
        if (module == null) return false;
        return module.isIn(state);
    }

    static void addRequirement(Requirement req) {
        requirements.add(req);
    }

    static void addForbidden(Forbidden forb) {
        forbiddens.add(forb);
    }

    static void addMutex(MutexGroup mutex) {
        mutexGroups.add(mutex);
    }

    static class Requirement {
        final State targetState;
        final BooleanSupplier condition;

        Requirement(State targetState, BooleanSupplier condition) {
            this.targetState = targetState;
            this.condition = condition;
        }
    }

    static class Forbidden {
        final Class<? extends State> stateClass;
        final BooleanSupplier condition;

        Forbidden(Class<? extends State> stateClass, BooleanSupplier condition) {
            this.stateClass = stateClass;
            this.condition = condition;
        }
    }

    static class MutexGroup {
        final Set<State> states;

        MutexGroup(Set<State> states) {
            this.states = states;
        }
    }

    /**
     * Functional interface for defining constraints using a builder pattern.
     */
    @FunctionalInterface
    public interface ConstraintDefiner {
        void define(ConstraintBuilder builder);
    }

    /**
     * Functional interface for handling constraint violations.
     * Implementations decide what happens when a state transition is blocked.
     */
    @FunctionalInterface
    public interface ViolationHandler {
        void handle(String reason, State attemptedState);

        /** Silently blocks the transition without any action. */
        ViolationHandler BLOCK = (reason, state) -> {};

        /** Logs the violation to stderr before blocking. */
        ViolationHandler LOG = (reason, state) -> 
            System.err.println("[Constraint] Blocked " + state.getClass().getSimpleName() + ": " + reason);

        /** Throws a {@link ConstraintViolationException} on violation. */
        ViolationHandler THROW = (reason, state) -> {
            throw new ConstraintViolationException(reason);
        };
    }

    /**
     * Exception thrown when a constraint violation occurs with THROW handler.
     */
    public static class ConstraintViolationException extends RuntimeException {
        public ConstraintViolationException(String message) {
            super(message);
        }
    }

    /**
     * Result of a constraint check.
     * Either {@link #ALLOWED} or a {@link Blocked} instance with a reason.
     */
    public static abstract class ConstraintResult {
        /** Singleton indicating the transition is allowed. */
        public static final ConstraintResult ALLOWED = new ConstraintResult() {};

        /**
         * Indicates a blocked transition with the reason for blocking.
         */
        public static class Blocked extends ConstraintResult {
            public final String reason;

            Blocked(String reason) {
                this.reason = reason;
            }
        }
    }

    /**
     * Builder for defining constraints within a {@link ConstraintDefiner}.
     */
    public static class ConstraintBuilder {
        /**
         * Creates a mutex group preventing any two states from being active simultaneously.
         * @param states the states that are mutually exclusive
         */
        public void mutex(State... states) {
            Constraints.addMutex(new MutexGroup(new HashSet<>(Arrays.asList(states))));
        }

        /**
         * Begins defining a requirement for the given state.
         * @param state the state that will have a requirement
         * @return a builder to complete the requirement definition
         */
        public RequirementBuilder require(State state) {
            return new RequirementBuilder(state);
        }

        /**
         * Begins defining a forbidden condition for a state type.
         * @param stateClass the class of states to forbid
         * @return a builder to complete the forbidden definition
         */
        public <T extends State> ForbiddenBuilder<T> forbid(Class<T> stateClass) {
            return new ForbiddenBuilder<>(stateClass);
        }
    }

    /**
     * Builder for defining requirements on a specific state.
     */
    public static class RequirementBuilder {
        private final State state;

        RequirementBuilder(State state) {
            this.state = state;
        }

        /**
         * Requires the condition to be true for this state to become active.
         * @param condition the condition that must be satisfied
         */
        public void when(BooleanSupplier condition) {
            Constraints.addRequirement(new Requirement(state, condition));
        }

        /**
         * Requires another state to be active for this state to become active.
         * @param requiredState the state that must be active
         */
        public void requires(State requiredState) {
            Constraints.addRequirement(new Requirement(state, () -> {
                Module module = requiredState.getModule();
                return module != null && module.isIn(requiredState);
            }));
        }

        /**
         * Requires at least one of the given states to be active.
         * @param requiredStates the states, at least one of which must be active
         */
        public void requiresAny(State... requiredStates) {
            Constraints.addRequirement(new Requirement(state, () -> {
                for (State s : requiredStates) {
                    Module module = s.getModule();
                    if (module != null && module.isIn(s)) {
                        return true;
                    }
                }
                return false;
            }));
        }
    }

    /**
     * Builder for defining forbidden conditions on a state type.
     * @param <T> the type of state being forbidden
     */
    public static class ForbiddenBuilder<T extends State> {
        private final Class<T> stateClass;

        ForbiddenBuilder(Class<T> stateClass) {
            this.stateClass = stateClass;
        }

        /**
         * Forbids this state type when the condition is true.
         * @param condition the condition under which the state is forbidden
         */
        public void when(BooleanSupplier condition) {
            Constraints.addForbidden(new Forbidden(stateClass, condition));
        }
    }
}
