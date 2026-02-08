package org.firstinspires.ftc.teamcode.architecture.layers;

import org.firstinspires.ftc.teamcode.architecture.layers.suppliers.CustomGamepad;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CircularLayeringSystem implements LayeringSystem<Integer> {
    private final List<CustomGamepad> gamepadList;
    private int currentIndex;

    @SafeVarargs
    public CircularLayeringSystem(CustomGamepad... gamepads) {
        if (gamepads == null || gamepads.length == 0) {
            throw new IllegalArgumentException("At least one gamepad needed");
        }

        this.gamepadList = new ArrayList<>(Arrays.asList(gamepads));
        this.currentIndex = 0;

        for (CustomGamepad gamepad : gamepadList) {
            if (gamepad != null) {
                gamepad.setAtRest(true);
            }
        }

        update();
    }

    public CircularLayeringSystem(List<CustomGamepad> gamepadList) {
        if (gamepadList == null || gamepadList.isEmpty()) {
            throw new IllegalArgumentException("Gamepad list cannot be null or empty");
        }

        this.gamepadList = new ArrayList<>(gamepadList);
        this.currentIndex = 0;

        for (CustomGamepad gamepad : this.gamepadList) {
            if (gamepad != null) {
                gamepad.setAtRest(true);
            }
        }

        update();
    }

    @Override
    public void setLayer(Integer index) {
        if (index == null) {
            throw new IllegalArgumentException("Index cannot be null");
        }

        if (gamepadList.isEmpty()) {
            throw new IllegalStateException("No gamepads available");
        }

        this.currentIndex =
                ((index % gamepadList.size()) + gamepadList.size()) % gamepadList.size();
        update();
    }

    @Override
    public Integer getLayer() {
        return currentIndex;
    }

    @Override
    public CustomGamepad getGamepad() {
        if (gamepadList.isEmpty()) {
            return null;
        }
        return gamepadList.get(currentIndex);
    }

    @Override
    public boolean isActive(CustomGamepad gamepad) {
        if (gamepad == null || gamepadList.isEmpty()) {
            return false;
        }

        return gamepad.equals(gamepadList.get(currentIndex));
    }

    public boolean isActive(int index) {
        if (index < 0 || index >= gamepadList.size()) {
            return false;
        }
        return index == currentIndex;
    }

    @Override
    public void update() {
        for (int i = 0; i < gamepadList.size(); i++) {
            CustomGamepad gamepad = gamepadList.get(i);
            if (gamepad != null) {
                gamepad.setAtRest(i != currentIndex);
            }
        }
    }

    public int next() {
        if (gamepadList.isEmpty()) {
            throw new IllegalStateException("No gamepads available");
        }

        currentIndex = (currentIndex + 1) % gamepadList.size();
        update();
        return currentIndex;
    }

    public int previous() {
        if (gamepadList.isEmpty()) {
            throw new IllegalStateException("No gamepads available");
        }

        currentIndex = (currentIndex - 1 + gamepadList.size()) % gamepadList.size();
        update();
        return currentIndex;
    }

    public int jumpForward(int steps) {
        if (gamepadList.isEmpty()) {
            throw new IllegalStateException("No gamepads available");
        }

        currentIndex = (currentIndex + steps % gamepadList.size() + gamepadList.size())
                % gamepadList.size();
        update();
        return currentIndex;
    }

    public int jumpBackward(int steps) {
        if (gamepadList.isEmpty()) {
            throw new IllegalStateException("No gamepads available");
        }

        currentIndex = (currentIndex - steps % gamepadList.size() + gamepadList.size())
                % gamepadList.size();
        update();
        return currentIndex;
    }

    public int getNextIndex() {
        if (gamepadList.isEmpty()) {
            return -1;
        }
        return (currentIndex + 1) % gamepadList.size();
    }

    public int getPreviousIndex() {
        if (gamepadList.isEmpty()) {
            return -1;
        }
        return (currentIndex - 1 + gamepadList.size()) % gamepadList.size();
    }

    public void insertGamepad(int index, CustomGamepad gamepad) {
        if (gamepad == null) {
            throw new IllegalArgumentException("Gamepad cannot be null");
        }

        if (index < 0 || index > gamepadList.size()) {
            throw new IndexOutOfBoundsException("Index " + index + " out of bounds");
        }

        gamepadList.add(index, gamepad);
        gamepad.setAtRest(true);

        if (index <= currentIndex) {
            currentIndex++;
        }

        update();
    }

    public CustomGamepad removeGamepad(int index) {
        if (index < 0 || index >= gamepadList.size()) {
            throw new IndexOutOfBoundsException("Index " + index + " out of bounds");
        }

        if (gamepadList.size() == 1) {
            throw new IllegalStateException("Cannot remove the last gamepad");
        }

        CustomGamepad removed = gamepadList.remove(index);

        if (index < currentIndex) {
            currentIndex--;
        } else if (index == currentIndex) {
            currentIndex = currentIndex % gamepadList.size();
        }

        update();
        return removed;
    }

    public boolean isAtIndex(int index) {
        return currentIndex
                == ((index % gamepadList.size()) + gamepadList.size()) % gamepadList.size();
    }

    public void invalidateAll() {
        for (CustomGamepad gamepad : gamepadList) {
            if (gamepad != null) {
                gamepad.invalidateAll();
            }
        }
    }

    public int getLayerCount() {
        return gamepadList.size();
    }

    public void addGamepad(CustomGamepad gamepad) {
        if (gamepad == null) {
            throw new IllegalArgumentException("Gamepad cannot be null");
        }

        gamepadList.add(gamepad);
        gamepad.setAtRest(true);
        update();
    }

    @Override
    public String toString() {
        return "CircularLayeringSystem{currentIndex=" + currentIndex
                + ", totalLayers=" + gamepadList.size() + ", nextIndex=" + getNextIndex()
                + ", previousIndex=" + getPreviousIndex() + "}";
    }
}
