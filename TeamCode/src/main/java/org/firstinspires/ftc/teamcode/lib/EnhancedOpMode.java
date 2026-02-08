package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.lang.reflect.Field;
import java.util.*;

public abstract class EnhancedOpMode extends OpMode {
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime gameTimer = new ElapsedTime();
    private final double[] loopTimes = new double[20];
    private int loopIndex = 0;

    private final List<Module> modules = new ArrayList<>();
    private VoltageSensor voltageSensor;
    private double voltage = 12.0;
    private boolean running = false;
    private boolean stopRequested = false;
    private boolean isInit = false;

    /** Minimum loop time in milliseconds. Set to stabilize control loop timing. */
    protected int minLoopMs = 0;
    /** Enable automatic voltage compensation for motors and CR servos. */
    protected boolean voltageCompensationEnabled = true;

    /** Override to return true if hardware writes should occur during init_loop. */
    protected boolean shouldWriteDuringInit() {
        return false;
    }

    @Override
    public final void init() {
        // Configure Lynx hubs for manual bulk caching to reduce read latency
        configureBulkCaching();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Allow subclasses to initialize hardware
        setup();
        // Automatically discover and register all Module fields
        autoDiscoverModules();
        // Initialize all registered modules
        initModules();
        onInit();

        loopTimer.reset();
        gameTimer.reset();

        isInit = true;
    }

    @Override
    public final void init_loop() {
        if (!isInit) return;

        if (stopRequested) {
            requestOpModeStop();
            return;
        }
        // Refresh cached hardware reads from the previous cycle
        clearBulkCaches();
        if (voltageCompensationEnabled) {
            updateVoltage();
            EnhancedMotor.updateVoltage(voltage);
            EnhancedCRServo.updateVoltage(voltage);
        }
        // Read sensor inputs
        readModules();
        onInitLoop();
        if (shouldWriteDuringInit())
            // Optional: write outputs during init phase
            writeModules();
        updateTelemetry();
        loopTimer.reset();
    }

    @Override
    public final void start() {
        running = true;
        gameTimer.reset();
        // Clear any pending actions from prior cycles
        Actions.reset();
        // Schedule default actions for each module
        scheduleDefaultActions();
        onStart();
        loopTimer.reset();
    }

    @Override
    public final void loop() {
        if (stopRequested) {
            running = false;
            requestOpModeStop();
            return;
        }

        clearBulkCaches();
        if (voltageCompensationEnabled) {
            updateVoltage();
            EnhancedMotor.updateVoltage(voltage);
            EnhancedCRServo.updateVoltage(voltage);
        }
        // Read sensor inputs
        readModules();
        onLoop();
        // Write command outputs to hardware
        writeModules();
        updateTelemetry();

        // Enforce minimum loop time to stabilize control loops
        long remaining = minLoopMs - (long) loopTimer.milliseconds();
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        recordLoopTime();
        loopTimer.reset();
    }

    @Override
    public final void stop() {
        running = false;
        // Clean up active actions and constraints
        Actions.shutdown();
        Constraints.clear();
        onStop();
    }

    /** Called once during init to set up hardware. */
    protected abstract void setup();

    /** Called after setup() and module initialization. */
    protected void onInit() {}

    /** Called repeatedly during init phase. */
    protected void onInitLoop() {}

    /** Called once when the OpMode starts. */
    protected void onStart() {}

    /** Called repeatedly during the main loop. */
    protected void onLoop() {}

    /** Called when the OpMode stops. */
    protected void onStop() {}

    /** Override to add custom telemetry each loop. */
    protected void telemetry() {}

    /**
     * Register modules manually. Modules are also auto-discovered from fields.
     */
    protected void register(Module... mods) {
        for (Module m : mods) {
            if (!modules.contains(m)) {
                modules.add(m);
            }
        }
    }

    private void autoDiscoverModules() {
        Set<Object> visited = new HashSet<>();
        try {
            // Recursively scan all fields to find Module instances
            discover(this, getClass(), visited);
        } catch (IllegalAccessException e) {
            throw new RuntimeException("Module auto-discovery failed", e);
        }
    }

    private void discover(Object obj, Class<?> clazz, Set<Object> visited)
            throws IllegalAccessException {
        if (obj == null || !visited.add(obj))
            return;

        while (clazz != EnhancedOpMode.class && clazz != Object.class && clazz != null) {
            for (Field f : clazz.getDeclaredFields()) {
                f.setAccessible(true);
                Object val = f.get(obj);
                if (val == null)
                    continue;

                if (val instanceof Module) {
                    register((Module) val);
                } else if (shouldRecurse(val.getClass())) {
                    // Recursively search objects, but skip standard library classes
                    discover(val, val.getClass(), visited);
                }
            }
            clazz = clazz.getSuperclass();
        }
    }

    private boolean shouldRecurse(Class<?> c) {
        String n = c.getName();
        return !n.startsWith("java.") && !n.startsWith("android.") && !n.startsWith("com.qualcomm.")
                && !n.startsWith("kotlin.");
    }

    private void initModules() {
        for (Module m : modules) {
            m.setTelemetry(telemetry);
            m.init();
        }
    }

    private void readModules() {
        for (Module m : modules) {
            m.read();
        }
    }

    protected void writeModules() {
        for (Module m : modules) {
            if (m.isWriteEnabled()) {
                m.write();
            }
        }
    }

    private void scheduleDefaultActions() {
        for (Module m : modules) {
            Action def = m.getDefaultAction();
            if (def != null && !Actions.isModuleActive(m)) {
                def.run();
            }
        }
    }

    private void configureBulkCaching() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void clearBulkCaches() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }
    }

    private void updateVoltage() {
        voltage = voltageSensor.getVoltage();
    }

    private void recordLoopTime() {
        loopTimes[loopIndex] = loopTimer.milliseconds();
        loopIndex = (loopIndex + 1) % loopTimes.length;
    }

    private void updateTelemetry() {
        if (telemetry instanceof EnhancedTelemetry) {
            EnhancedTelemetry et = (EnhancedTelemetry) telemetry;
            
            // Add timing to Dashboard only
            et.addDashboardData("Game Time", "%.1fs", gameTimer.seconds());
            et.addDashboardData("Loop Time", "%.1fms (avg %.1fms)", loopTimer.milliseconds(), avgLoopMs());
            
            // Modules group
            if (!modules.isEmpty()) {
                et.addSeparator();
                et.addGroupHeader("MODULES", EnhancedTelemetry.COLOR_MODULE);
                for (Module m : modules) {
                    m.telemetry();
                }
            }
        } else {
            // Fallback for non-enhanced telemetry
            telemetry.addData("Game Time", "%.1fs", gameTimer.seconds());
            telemetry.addData("Loop Time", "%.1fms (avg %.1fms)", loopTimer.milliseconds(), avgLoopMs());
            
            for (Module m : modules) {
                m.telemetry();
            }
        }

        telemetry();
        telemetry.update();
    }

    /** Request a graceful stop of the OpMode. */
    public final void requestStop() {
        stopRequested = true;
    }

    /** Check if the OpMode is currently in the main loop phase. */
    public final boolean isRunning() {
        return running;
    }

    /** Get the current battery voltage. */
    public final double getVoltage() {
        return voltage;
    }

    /** Get the timer tracking total game time. */
    public final ElapsedTime getGameTimer() {
        return gameTimer;
    }

    /** Get the average loop time in milliseconds. */
    public final double avgLoopMs() {
        double sum = 0;
        int count = 0;
        for (double t : loopTimes) {
            if (t > 0) {
                sum += t;
                count++;
            }
        }
        return count > 0 ? sum / count : 0;
    }

    /** Get the total current draw from all Lynx hubs in amps. */
    public final double getTotalCurrent() {
        double total = 0;
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            total += hub.getCurrent(CurrentUnit.AMPS);
        }
        return total;
    }

    /** Get an unmodifiable list of all registered modules. */
    public final List<Module> getModules() {
        return Collections.unmodifiableList(modules);
    }
}
