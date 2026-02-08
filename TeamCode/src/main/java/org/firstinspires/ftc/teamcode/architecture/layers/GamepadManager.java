package org.firstinspires.ftc.teamcode.architecture.layers;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.architecture.layers.suppliers.CustomGamepad;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class GamepadManager {
    private GamepadManager instance;

    private LayeredGamepad<?> gamepad1;
    private LayeredGamepad<?> gamepad2;

    private final Map<String, LayeredGamepad<?>> gamepadRegistry = new ConcurrentHashMap<>();
    private final Map<String, CustomGamepad> customGamepadRegistry = new ConcurrentHashMap<>();

    private GamepadManager() {}

    public GamepadManager getInstance() {
        if (instance == null) {
            instance = new GamepadManager();
        }
        return instance;
    }

    public LayeredGamepad<?> gamepad1() {
        return gamepad1;
    }

    public LayeredGamepad<?> gamepad2() {
        return gamepad2;
    }

    public <T> void initializeGamepad1(LayeringSystem<T> layeringSystem) {
        gamepad1 = new LayeredGamepad<>(layeringSystem);
        getInstance().registerGamepad("gamepad1", gamepad1);
    }

    public <T> void initializeGamepad2(LayeringSystem<T> layeringSystem) {
        gamepad2 = new LayeredGamepad<>(layeringSystem);
        getInstance().registerGamepad("gamepad2", gamepad2);
    }

    public static CustomGamepad createCustomGamepad(Gamepad gamepad) {
        return new CustomGamepad(gamepad);
    }

    public void registerGamepad(String name, LayeredGamepad<?> gamepad) {
        if (name == null || name.isEmpty()) {
            throw new IllegalArgumentException("Gamepad name cannot be null or empty");
        }
        gamepadRegistry.put(name, gamepad);
    }

    public void registerCustomGamepad(String name, CustomGamepad gamepad) {
        if (name == null || name.isEmpty()) {
            throw new IllegalArgumentException("Gamepad name cannot be null or empty");
        }
        customGamepadRegistry.put(name, gamepad);
    }

    public LayeredGamepad<?> getGamepad(String name) {
        return gamepadRegistry.get(name);
    }

    public CustomGamepad getCustomGamepad(String name) {
        return customGamepadRegistry.get(name);
    }

    public void updateAll() {
        for (LayeredGamepad<?> gamepad : gamepadRegistry.values()) {
            if (gamepad != null) {
                gamepad.update();
                gamepad.invalidateAll();
            }
        }

        for (CustomGamepad gamepad : customGamepadRegistry.values()) {
            if (gamepad != null) {
                gamepad.invalidateAll();
            }
        }
    }

    public void setAllAtRest(boolean atRest) {
        for (CustomGamepad gamepad : customGamepadRegistry.values()) {
            if (gamepad != null) {
                gamepad.setAtRest(atRest);
            }
        }
    }

    public int getLayeredGamepadCount() {
        return gamepadRegistry.size();
    }

    public int getCustomGamepadCount() {
        return customGamepadRegistry.size();
    }

    public boolean hasLayeredGamepad(String name) {
        return gamepadRegistry.containsKey(name);
    }

    public boolean hasCustomGamepad(String name) {
        return customGamepadRegistry.containsKey(name);
    }

    public LayeredGamepad<?> unregisterGamepad(String name) {
        LayeredGamepad<?> removed = gamepadRegistry.remove(name);

        if ("gamepad1".equals(name)) {
            gamepad1 = null;
        } else if ("gamepad2".equals(name)) {
            gamepad2 = null;
        }

        return removed;
    }

    public CustomGamepad unregisterCustomGamepad(String name) {
        return customGamepadRegistry.remove(name);
    }

    public void clearAll() {
        gamepadRegistry.clear();
        customGamepadRegistry.clear();
        gamepad1 = null;
        gamepad2 = null;
    }

    public String getDebugInfo() {
        StringBuilder sb = new StringBuilder();
        sb.append("GamepadManager Debug Info:\n");
        sb.append("Layered Gamepads: ").append(gamepadRegistry.size()).append("\n");
        sb.append("Custom Gamepads: ").append(customGamepadRegistry.size()).append("\n");
        sb.append("Gamepad1 initialized: ").append(gamepad1 != null).append("\n");
        sb.append("Gamepad2 initialized: ").append(gamepad2 != null).append("\n");

        if (!gamepadRegistry.isEmpty()) {
            sb.append("Registered Layered Gamepads: ");
            sb.append(String.join(", ", gamepadRegistry.keySet()));
            sb.append("\n");
        }

        if (!customGamepadRegistry.isEmpty()) {
            sb.append("Registered Custom Gamepads: ");
            sb.append(String.join(", ", customGamepadRegistry.keySet()));
            sb.append("\n");
        }

        return sb.toString();
    }
}
