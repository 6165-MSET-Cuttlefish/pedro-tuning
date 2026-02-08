package org.firstinspires.ftc.teamcode.architecture.hardware;

public class SRSHubAbsoluteAnalogEncoder extends AbsoluteAnalogEncoder {
    private final SRSHubManager srsHubManager;
    private final int pin;

    public SRSHubAbsoluteAnalogEncoder(SRSHubManager srsHubManager, int pin) {
        this(srsHubManager, pin, 0.0, 1.0, false);
    }

    public SRSHubAbsoluteAnalogEncoder(SRSHubManager srsHubManager, int pin, double offset,
            double gearRatio, boolean inverted) {
        super(null, offset, gearRatio, inverted);
        this.srsHubManager = srsHubManager;
        this.pin = pin;
    }

    @Override
    protected double getVoltageAsAngle() {
        double normalizedValue = srsHubManager.readAnalogDigitalDevice(pin);
        return normalizedValue * 360.0;
    }

    @Override
    public double getVoltage() {
        return srsHubManager.readAnalogDigitalDevice(pin);
    }

    public double getNormalizedValue() {
        return srsHubManager.readAnalogDigitalDevice(pin);
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "SRSHub Absolute Analog Encoder";
    }

    @Override
    public String getConnectionInfo() {
        return "SRSHub Pin " + pin;
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {}

    @Override
    public void close() {}
}
