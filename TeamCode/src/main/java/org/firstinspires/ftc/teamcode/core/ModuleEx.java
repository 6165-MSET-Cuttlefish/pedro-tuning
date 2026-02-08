package org.firstinspires.ftc.teamcode.core;

import org.firstinspires.ftc.teamcode.lib.EnhancedTelemetry;
import org.firstinspires.ftc.teamcode.lib.Module;

/**
 * Extended Module base class with EnhancedTelemetry integration.
 * Provides convenience methods for logging to Driver Station and FTC Dashboard separately.
 * 
 * Usage:
 *   - Use log() for both outputs (inherited from Module)
 *   - Use logDS() for Driver Station only
 *   - Use logDashboard() for FTC Dashboard only
 *   - Chain methods: logDSChain(), logDashboardChain()
 */
public abstract class ModuleEx extends Module {

    protected final EnhancedTelemetry getEnhancedTelemetry() {
        if (getTelemetry() instanceof EnhancedTelemetry) {
            return (EnhancedTelemetry) getTelemetry();
        }
        return null;
    }

    protected final void logDS(String caption, Object value) {
        EnhancedTelemetry et = getEnhancedTelemetry();
        if (et != null) {
            et.addDSData(getName() + " " + caption, value);
        } else {
            log(caption, value);
        }
    }

    protected final void logDS(String caption, String format, Object... args) {
        EnhancedTelemetry et = getEnhancedTelemetry();
        if (et != null) {
            et.addDSData(getName() + " " + caption, format, args);
        } else {
            log(caption, format, args);
        }
    }

    protected final void logDashboard(String caption, Object value) {
        EnhancedTelemetry et = getEnhancedTelemetry();
        if (et != null) {
            et.addDashboardData(getName() + " " + caption, value);
        } else {
            log(caption, value);
        }
    }

    protected final void logDashboard(String caption, String format, Object... args) {
        EnhancedTelemetry et = getEnhancedTelemetry();
        if (et != null) {
            et.addDashboardData(getName() + " " + caption, format, args);
        } else {
            log(caption, format, args);
        }
    }

    protected final ModuleEx logDSChain(String caption, Object value) {
        logDS(caption, value);
        return this;
    }

    protected final ModuleEx logDSChain(String caption, String format, Object... args) {
        logDS(caption, format, args);
        return this;
    }

    protected final ModuleEx logDashboardChain(String caption, Object value) {
        logDashboard(caption, value);
        return this;
    }

    protected final ModuleEx logDashboardChain(String caption, String format, Object... args) {
        logDashboard(caption, format, args);
        return this;
    }
}
