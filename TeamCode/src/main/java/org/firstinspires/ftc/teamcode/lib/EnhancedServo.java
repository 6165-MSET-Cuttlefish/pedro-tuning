package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Enhanced Servo wrapper with position caching and bounds.
 * Use method chaining to configure: new EnhancedServo(servo).withCachingTolerance(0.001).withPositionBounds(0.1, 0.9);
 */
public class EnhancedServo implements Servo, PwmControl {
    private final ServoImplEx servo;

    private double cachingTolerance = 0.0;
    private double cachedPosition = Double.NaN;

    private double minPosition = 0.0;
    private double maxPosition = 1.0;

    public EnhancedServo(ServoImplEx servo) {
        this.servo = servo;
    }

    public EnhancedServo(HardwareMap hardwareMap, String name) {
        this.servo = hardwareMap.get(ServoImplEx.class, name);
    }

    /**
     * Set how much position must change before writing to hardware.
     * Reduces communication traffic and improves loop times.
     * @param tolerance Position change threshold (0.0 to 1.0)
     */
    public EnhancedServo withCachingTolerance(double tolerance) {
        this.cachingTolerance = tolerance;
        return this;
    }

    /**
     * Clamp all position values to the given range.
     * @param min Minimum position (0.0 to 1.0)
     * @param max Maximum position (0.0 to 1.0)
     */
    public EnhancedServo withPositionBounds(double min, double max) {
        this.minPosition = min;
        this.maxPosition = max;
        return this;
    }

    /**
     * Set position with caching.
     * Only writes to hardware if the change exceeds the caching tolerance.
     */
    @Override
    public void setPosition(double position) {
        double corrected = Math.max(minPosition, Math.min(maxPosition, position));
        if (shouldUpdatePosition(corrected)) {
            cachedPosition = corrected;
            servo.setPosition(corrected);
        }
    }

    /**
     * Set position directly without caching.
     * Useful for testing or special cases.
     */
    public void setPositionRaw(double position) {
        double corrected = Math.max(minPosition, Math.min(maxPosition, position));
        cachedPosition = corrected;
        servo.setPosition(corrected);
    }

    private boolean shouldUpdatePosition(double newPosition) {
        return Math.abs(newPosition - cachedPosition) >= cachingTolerance
                || Double.isNaN(cachedPosition);
    }

    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection();
    }

    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction);
    }

    @Override
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }

    @Override
    public Manufacturer getManufacturer() {
        return servo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return servo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return servo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        servo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        servo.close();
    }

    @Override
    public double getPosition() {
        return servo.getPosition();
    }

    /** Set the caching tolerance. Clamped to [0, 1]. */
    public void setCachingTolerance(double tolerance) {
        cachingTolerance = Math.max(0.0, Math.min(1.0, tolerance));
    }

    /** Get the current caching tolerance. */
    public double getCachingTolerance() {
        return cachingTolerance;
    }

    /** Get the underlying ServoImplEx for advanced operations. */
    public ServoImplEx getUnderlying() {
        return servo;
    }

    /** Get the last position value written to the servo. */
    public double getCachedPosition() {
        return cachedPosition;
    }

    @Override
    public void setPwmRange(PwmRange range) {
        servo.setPwmRange(range);
    }

    @Override
    public PwmRange getPwmRange() {
        return servo.getPwmRange();
    }

    @Override
    public void setPwmEnable() {
        servo.setPwmEnable();
    }

    @Override
    public void setPwmDisable() {
        servo.setPwmDisable();
    }

    @Override
    public boolean isPwmEnabled() {
        return servo.isPwmEnabled();
    }
}
