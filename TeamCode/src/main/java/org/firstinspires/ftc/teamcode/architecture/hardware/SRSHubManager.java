package org.firstinspires.ftc.teamcode.architecture.hardware;

import static org.firstinspires.ftc.teamcode.core.Robot.srsHubTelemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.ModuleEx;

public class SRSHubManager extends ModuleEx {
    private final SRSHub hub;

    private final SRSHub.Config config;

    private final String deviceName;

    private boolean initialized = false;

    private boolean disconnected = false;

    private boolean verboseTelemetry = false;

    public SRSHubManager(HardwareMap hardwareMap, String deviceName) {
        super();

        this.deviceName = deviceName;
        this.hub = hardwareMap.get(SRSHub.class, deviceName);
        this.config = new SRSHub.Config();
    }

    public SRSHubManager(HardwareMap hardwareMap) {
        this(hardwareMap, "srsHub");
    }

    public void configureAnalogDigitalDevice(int pin, SRSHub.AnalogDigitalDevice device) {
        if (initialized) {
            throw new IllegalStateException("Cannot configure SRSHub after initialization. "
                    + "Call configuration methods before initialize().");
        }
        config.setAnalogDigitalDevice(pin, device);
    }

    public void configureEncoder(int port, SRSHub.Encoder encoder) {
        if (initialized) {
            throw new IllegalStateException("Cannot configure SRSHub after initialization. "
                    + "Call configuration methods before initialize().");
        }
        config.setEncoder(port, encoder);
    }

    public void addI2CDevice(int bus, SRSHub.I2CDevice device) {
        if (initialized) {
            throw new IllegalStateException("Cannot configure SRSHub after initialization. "
                    + "Call configuration methods before initialize().");
        }
        config.addI2CDevice(bus, device);
    }

    public void initialize() {
        if (initialized) {
            throw new IllegalStateException("SRSHub already initialized");
        }

        hub.init(config);
        while (!hub.ready());
        initialized = true;
    }

    @Override
    protected void initStates() {}

    @Override
    protected void read() {

        if (!initialized) {
            return;
        }

        // Update all sensor readings from the hub
        hub.update();

        // Track connection status
        disconnected = hub.disconnected();
    }

    @Override
    protected void write() {}

    public double readAnalogDigitalDevice(int pin) {
        checkInitialized();
        return hub.readAnalogDigitalDevice(pin);
    }

    public SRSHub.PosVel readEncoder(int port) {
        checkInitialized();
        return hub.readEncoder(port);
    }

    public <T extends SRSHub.I2CDevice> T getI2CDevice(int bus, Class<T> deviceClass) {
        checkInitialized();
        return hub.getI2CDevice(bus, deviceClass);
    }

    public void runCommand(SRSHub.Command command) {
        checkInitialized();
        hub.runCommand(command);
    }

    public boolean isReady() {
        return initialized && hub.ready();
    }

    public boolean isDisconnected() {
        return disconnected;
    }

    public void setVerboseTelemetry(boolean verbose) {
        this.verboseTelemetry = verbose;
    }

    @Override
    protected void onTelemetry() {
        if (!srsHubTelemetry.TOGGLE) {
            return;
        }

        if (!initialized) {
            logDashboard("Status", "NOT INITIALIZED");
            return;
        }

        if (!isReady()) {
            logDashboard("Status", "INITIALIZING...");
            return;
        }

        if (disconnected) {
            logDashboard("Status", "⚠ DISCONNECTED");
        } else {
            logDashboard("Status", "✓ Connected");
        }

        if (verboseTelemetry && !disconnected) {
        }
    }

    private void checkInitialized() {
        if (!initialized) {
            throw new IllegalStateException("SRSHub must be initialized before reading. "
                    + "Call initialize() after configuration.");
        }
    }

    public SRSHub getHub() {
        return hub;
    }
}
