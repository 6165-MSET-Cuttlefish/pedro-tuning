package org.firstinspires.ftc.teamcode.architecture.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;

public class AbsoluteAnalogEncoder implements HardwareDevice {
    protected final AnalogInput analogInput;
    protected double offset;
    protected final double gearRatio;
    protected final boolean inverted;

    private double lastOutputPosition;
    private int revolutionCount;
    private boolean firstReading;
    public double delta;

    public AbsoluteAnalogEncoder(AnalogInput analogInput) {
        this(analogInput, 0.0, 1.0, false);
    }

    public AbsoluteAnalogEncoder(
            AnalogInput analogInput, double offset, double gearRatio, boolean inverted) {
        this.analogInput = analogInput;
        this.offset = offset;
        this.gearRatio = gearRatio;
        this.inverted = inverted;
        this.lastOutputPosition = 0.0;
        this.revolutionCount = 0;
        this.firstReading = true;
    }

    public double getPosition() {
        double encoderPosition = getRawEncoderPosition();
        if (firstReading) {
            firstReading = false;
        } else {
            delta = encoderPosition - lastOutputPosition;
            if (delta < -180.0) {
                revolutionCount++;
            } else if (delta > 180.0) {
                revolutionCount--;
            }
        }
        lastOutputPosition = encoderPosition;
        double totalEncoderAngle = encoderPosition + (360.0 * revolutionCount) + offset;
        return totalEncoderAngle * gearRatio;
    }

    public double getRawEncoderPosition() {
        double encoderPosition = getVoltageAsAngle();

        if (inverted) {
            encoderPosition = 360.0 - encoderPosition;
        }

        encoderPosition = encoderPosition % 360.0;
        if (encoderPosition < 0) {
            encoderPosition += 360.0;
        }

        return encoderPosition;
    }

    protected double getVoltageAsAngle() {
        double voltage = analogInput.getVoltage();
        double maxVoltage = analogInput.getMaxVoltage();
        return (voltage / maxVoltage) * 360.0;
    }

    public double getDelta() {
        return delta;
    }

    public int getRevolutionCount() {
        return revolutionCount;
    }

    public double getVoltage() {
        return analogInput.getVoltage();
    }

    public double getNormalizedValue() {
        if (analogInput == null) {
            return Double.NaN;
        }
        double maxVoltage = analogInput.getMaxVoltage();
        if (maxVoltage == 0.0) {
            return 0.0;
        }
        return analogInput.getVoltage() / maxVoltage;
    }

    public void zero() {
        offset = -getRawEncoderPosition();
    }

    @Override
    public Manufacturer getManufacturer() {
        return analogInput.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return "Absolute Analog Encoder";
    }

    @Override
    public String getConnectionInfo() {
        return analogInput.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {}

    @Override
    public void close() {
        analogInput.close();
    }
}
