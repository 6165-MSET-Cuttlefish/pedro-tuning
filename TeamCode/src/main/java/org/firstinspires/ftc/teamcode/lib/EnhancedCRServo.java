package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.*;

/**
 * Enhanced CRServo wrapper with power caching, bounds, and voltage compensation.
 * Use method chaining to configure: new EnhancedCRServo(servo).withCachingTolerance(0.02).withVoltageCompensation(13.5);
 */
public class EnhancedCRServo implements CRServo, PwmControl {
    private final CRServoImplEx crServo;

    private double cachingTolerance = 0.0;
    private double cachedPower = Double.NaN;

    private double minPower = -1.0;
    private double maxPower = 1.0;

    private boolean voltageCompensationEnabled = false;
    private double referenceVoltage = 13.5;

    private static volatile double currentVoltage = 12.0;

    public EnhancedCRServo(CRServoImplEx crServo) {
        this.crServo = crServo;
    }

    public EnhancedCRServo(HardwareMap hardwareMap, String name) {
        this.crServo = hardwareMap.get(CRServoImplEx.class, name);
    }

    /**
     * Set how much power must change before writing to hardware.
     * Reduces communication traffic and improves loop times.
     * @param tolerance Power change threshold (0.0 to 2.0)
     */
    public EnhancedCRServo withCachingTolerance(double tolerance) {
        this.cachingTolerance = tolerance;
        return this;
    }

    /**
     * Clamp all power values to the given range.
     * @param min Minimum power (typically -1.0 or 0.0)
     * @param max Maximum power (typically 1.0)
     */
    public EnhancedCRServo withPowerBounds(double min, double max) {
        this.minPower = min;
        this.maxPower = max;
        return this;
    }

    /**
     * Enable voltage compensation to maintain consistent servo output as battery drains.
     * @param referenceVoltage The voltage at which power=1.0 produces full output (typically 13.5V)
     */
    public EnhancedCRServo withVoltageCompensation(double referenceVoltage) {
        this.voltageCompensationEnabled = true;
        this.referenceVoltage = referenceVoltage;
        return this;
    }

    /**
     * Update the current battery voltage for all EnhancedCRServo instances.
     * Called automatically by OpModeEx each loop.
     */
    public static void updateVoltage(double voltage) {
        currentVoltage = voltage;
    }

    /**
     * Set power with voltage compensation and caching.
     * Only writes to hardware if the change exceeds the caching tolerance.
     */
    @Override
    public void setPower(double power) {
        double compensated = applyVoltageCompensation(power);
        double corrected = Math.max(minPower, Math.min(maxPower, compensated));

        if (shouldUpdatePower(corrected)) {
            cachedPower = corrected;
            crServo.setPower(corrected);
        }
    }

    /**
     * Set power directly without voltage compensation.
     * Useful for testing or special cases.
     */
    public void setPowerRaw(double power) {
        double corrected = Math.max(minPower, Math.min(maxPower, power));
        cachedPower = corrected;
        crServo.setPower(corrected);
    }

    private boolean shouldUpdatePower(double newPower) {
        return Math.abs(newPower - cachedPower) >= cachingTolerance
                || (newPower == 0.0 && cachedPower != 0.0)
                || (newPower >= maxPower && !(cachedPower >= maxPower))
                || (newPower <= minPower && !(cachedPower <= minPower)) || Double.isNaN(cachedPower);
    }

    private double applyVoltageCompensation(double power) {
        if (!voltageCompensationEnabled || power == 0.0) {
            return power;
        }

        // Scale power up if voltage is below reference to maintain consistent performance
        if (currentVoltage >= referenceVoltage) {
            return power;
        }

        return power * (referenceVoltage / currentVoltage);
    }

    /** Set the caching tolerance. Clamped to [0, 1]. */
    public void setCachingTolerance(double tolerance) {
        cachingTolerance = Math.max(0.0, Math.min(1.0, tolerance));
    }

    /** Get the current caching tolerance. */
    public double getCachingTolerance() {
        return cachingTolerance;
    }

    /** Set the reference voltage for compensation. */
    public void setReferenceVoltage(double voltage) {
        referenceVoltage = voltage;
    }

    /** Get the reference voltage for compensation. */
    public double getReferenceVoltage() {
        return referenceVoltage;
    }

    /** Enable or disable voltage compensation. */
    public void setVoltageCompensationEnabled(boolean enabled) {
        voltageCompensationEnabled = enabled;
    }

    /** Check if voltage compensation is enabled. */
    public boolean isVoltageCompensationEnabled() {
        return voltageCompensationEnabled;
    }

    /** Get the underlying CRServoImplEx for advanced operations. */
    public CRServoImplEx getUnderlying() {
        return crServo;
    }

    /** Get the last power value written to the servo. */
    public double getCachedPower() {
        return cachedPower;
    }

    @Override
    public ServoController getController() {
        return crServo.getController();
    }

    @Override
    public int getPortNumber() {
        return crServo.getPortNumber();
    }

    @Override
    public Direction getDirection() {
        return crServo.getDirection();
    }

    @Override
    public void setDirection(Direction direction) {
        crServo.setDirection(direction);
    }

    @Override
    public double getPower() {
        return crServo.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return crServo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return crServo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return crServo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return crServo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        crServo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        crServo.close();
    }

    @Override
    public void setPwmRange(PwmRange range) {
        crServo.setPwmRange(range);
    }

    @Override
    public PwmRange getPwmRange() {
        return crServo.getPwmRange();
    }

    @Override
    public void setPwmEnable() {
        crServo.setPwmEnable();
    }

    @Override
    public void setPwmDisable() {
        crServo.setPwmDisable();
    }

    @Override
    public boolean isPwmEnabled() {
        return crServo.isPwmEnabled();
    }
}
