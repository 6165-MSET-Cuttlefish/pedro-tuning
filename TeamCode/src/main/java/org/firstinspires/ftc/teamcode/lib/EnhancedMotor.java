package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Enhanced DcMotorEx wrapper with power caching, bounds, and voltage compensation.
 * Use method chaining to configure: new EnhancedMotor(motor).withCachingTolerance(0.02).withVoltageCompensation(13.5);
 */
public class EnhancedMotor implements DcMotorEx {
    private final DcMotorEx motor;

    private double cachingTolerance = 0.0;
    private double cachedPower = Double.NaN;

    private double minPower = -1.0;
    private double maxPower = 1.0;

    private boolean voltageCompensationEnabled = false;
    private double referenceVoltage = 10;

    private static volatile double currentVoltage = 12.0;

    public EnhancedMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public EnhancedMotor(HardwareMap hardwareMap, String name) {
        this.motor = hardwareMap.get(DcMotorEx.class, name);
    }

    /**
     * Set how much power must change before writing to hardware.
     * Reduces communication traffic and improves loop times.
     * @param tolerance Power change threshold (0.0 to 2.0)
     */
    public EnhancedMotor withCachingTolerance(double tolerance) {
        this.cachingTolerance = tolerance;
        return this;
    }

    /**
     * Clamp all power values to the given range.
     * @param min Minimum power (typically -1.0 or 0.0)
     * @param max Maximum power (typically 1.0)
     */
    public EnhancedMotor withPowerBounds(double min, double max) {
        this.minPower = min;
        this.maxPower = max;
        return this;
    }

    /**
     * Enable voltage compensation to maintain consistent motor output as battery drains.
     * @param referenceVoltage The voltage at which power=1.0 produces full output (typically 13.5V)
     */
    public EnhancedMotor withVoltageCompensation(double referenceVoltage) {
        this.voltageCompensationEnabled = true;
        this.referenceVoltage = referenceVoltage;
        return this;
    }

    /**
     * Update the current battery voltage for all EnhancedMotor instances.
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
            motor.setPower(corrected);
        }
    }

    /**
     * Set power directly without voltage compensation.
     * Useful for testing or special cases.
     */
    public void setPowerRaw(double power) {
        double corrected = Math.max(minPower, Math.min(maxPower, power));
        cachedPower = corrected;
        motor.setPower(corrected);
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

    /** Get the underlying DcMotorEx for advanced operations. */
    public DcMotorEx getUnderlying() {
        return motor;
    }

    /** Get the last power value written to the motor. */
    public double getCachedPower() {
        return cachedPower;
    }

    @Override
    public void setMotorEnable() {
        motor.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        motor.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return motor.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        motor.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity();
    }

    @Override
    public void setVelocity(double angularRate) {
        motor.setVelocity(angularRate);
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return motor.getVelocity(unit);
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        motor.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) {
        motor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        motor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        motor.setPositionPIDFCoefficients(p);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return motor.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return motor.getPIDFCoefficients(mode);
    }

    @Override
    public int getTargetPositionTolerance() {
        return motor.getTargetPositionTolerance();
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        motor.setTargetPositionTolerance(tolerance);
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return motor.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        motor.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }

    @Override
    public RunMode getMode() {
        return motor.getMode();
    }

    @Override
    public void setMode(RunMode mode) {
        motor.setMode(mode);
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return motor.getController();
    }

    @Override
    public int getPortNumber() {
        return motor.getPortNumber();
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public void setPowerFloat() {
        motor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return motor.getPowerFloat();
    }

    @Override
    public boolean isBusy() {
        return motor.isBusy();
    }

    @Override
    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    @Override
    public void setTargetPosition(int position) {
        motor.setTargetPosition(position);
    }

    @Override
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public Direction getDirection() {
        return motor.getDirection();
    }

    @Override
    public void setDirection(Direction direction) {
        motor.setDirection(direction);
    }

    @Override
    public double getPower() {
        return motor.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        motor.close();
    }
}
