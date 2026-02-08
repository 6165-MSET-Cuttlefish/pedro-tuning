package org.firstinspires.ftc.teamcode.architecture.layers;

import org.firstinspires.ftc.teamcode.architecture.layers.suppliers.CustomGamepad;

import java.util.Map;
import java.util.Set;

public class MapLayeringSystem<T> implements LayeringSystem<T> {
    private final Map<T, CustomGamepad> gamepadMap;
    private T currentLayer;

    public MapLayeringSystem(T initialLayer, Map<T, CustomGamepad> gamepadMap) {
        if (gamepadMap == null || gamepadMap.isEmpty()) {
            throw new IllegalArgumentException("Gamepad map cannot be null or empty");
        }

        if (initialLayer != null && !gamepadMap.containsKey(initialLayer)) {
            throw new IllegalArgumentException(
                    "Initial layer " + initialLayer + " not found in gamepad map");
        }

        this.gamepadMap = gamepadMap;
        this.currentLayer = initialLayer;

        update();
    }

    @Override
    public void setLayer(T layer) {
        if (layer == null) {
            throw new IllegalArgumentException("Layer cannot be null");
        }

        if (!gamepadMap.containsKey(layer)) {
            throw new IllegalArgumentException("Layer " + layer + " not found in gamepad map");
        }

        this.currentLayer = layer;
        update();
    }

    @Override
    public T getLayer() {
        return currentLayer;
    }

    @Override
    public CustomGamepad getGamepad() {
        if (currentLayer == null) {
            return null;
        }
        return gamepadMap.get(currentLayer);
    }

    @Override
    public boolean isActive(CustomGamepad gamepad) {
        if (gamepad == null || currentLayer == null) {
            return false;
        }

        CustomGamepad activeGamepad = gamepadMap.get(currentLayer);
        return gamepad.equals(activeGamepad);
    }

    @Override
    public void update() {
        for (Map.Entry<T, CustomGamepad> entry : gamepadMap.entrySet()) {
            CustomGamepad gamepad = entry.getValue();
            if (gamepad != null) {
                boolean isActive = entry.getKey().equals(currentLayer);
                gamepad.setAtRest(!isActive);
            }
        }
    }

    public Set<T> getAvailableLayers() {
        return gamepadMap.keySet();
    }

    public boolean hasLayer(T layer) {
        return gamepadMap.containsKey(layer);
    }

    public void putLayer(T layer, CustomGamepad gamepad) {
        if (layer == null) {
            throw new IllegalArgumentException("Layer cannot be null");
        }
        if (gamepad == null) {
            throw new IllegalArgumentException("Gamepad cannot be null");
        }

        gamepadMap.put(layer, gamepad);

        if (currentLayer == null) {
            currentLayer = layer;
        }

        update();
    }

    public CustomGamepad removeLayer(T layer) {
        if (layer == null) {
            return null;
        }

        CustomGamepad removedGamepad = gamepadMap.remove(layer);

        if (layer.equals(currentLayer)) {
            if (gamepadMap.isEmpty()) {
                currentLayer = null;
            } else {
                currentLayer = gamepadMap.keySet().iterator().next();
            }
            update();
        }

        return removedGamepad;
    }

    public int getLayerCount() {
        return gamepadMap.size();
    }

    public boolean isEmpty() {
        return gamepadMap.isEmpty();
    }

    public void invalidateAll() {
        for (CustomGamepad gamepad : gamepadMap.values()) {
            if (gamepad != null) {
                gamepad.invalidateAll();
            }
        }
    }

    @Override
    public String toString() {
        return "";
    }
}
