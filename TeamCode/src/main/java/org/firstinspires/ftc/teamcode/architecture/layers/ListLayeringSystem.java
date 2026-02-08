package org.firstinspires.ftc.teamcode.architecture.layers;

import org.firstinspires.ftc.teamcode.architecture.layers.suppliers.CustomGamepad;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ListLayeringSystem implements LayeringSystem<Integer> {
    private final List<CustomGamepad> gamepadList;
    private int currentIndex;

    @SafeVarargs
    public ListLayeringSystem(CustomGamepad... gamepads) {
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

    public ListLayeringSystem(List<CustomGamepad> gamepadList) {
        if (gamepadList == null || gamepadList.isEmpty()) {
            throw new IllegalArgumentException("Gamepad null");
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

        if (index < 0 || index >= gamepadList.size()) {
            throw new IndexOutOfBoundsException("Index " + index
                    + " out of bounds. Valid range: 0 to " + (gamepadList.size() - 1));
        }

        this.currentIndex = index;
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

    public boolean next() {
        if (currentIndex < gamepadList.size() - 1) {
            currentIndex++;
            update();
            return true;
        }
        return false;
    }

    public boolean previous() {
        if (currentIndex > 0) {
            currentIndex--;
            update();
            return true;
        }
        return false;
    }

    public boolean hasNext() {
        return currentIndex < gamepadList.size() - 1;
    }

    public boolean hasPrevious() {
        return currentIndex > 0;
    }

    public int getLayerCount() {
        return gamepadList.size();
    }

    public int getNextIndex() {
        return hasNext() ? currentIndex + 1 : -1;
    }

    public int getPreviousIndex() {
        return hasPrevious() ? currentIndex - 1 : -1;
    }

    public boolean jumpBy(int steps) {
        int newIndex = currentIndex + steps;

        if (newIndex >= 0 && newIndex < gamepadList.size()) {
            currentIndex = newIndex;
            update();
            return true;
        }

        return false;
    }

    public void goToFirst() {
        if (!gamepadList.isEmpty()) {
            currentIndex = 0;
            update();
        }
    }

    public void goToLast() {
        if (!gamepadList.isEmpty()) {
            currentIndex = gamepadList.size() - 1;
            update();
        }
    }

    public boolean isAtFirst() {
        return currentIndex == 0;
    }

    public boolean isAtLast() {
        return currentIndex == gamepadList.size() - 1;
    }

    public void addGamepad(CustomGamepad gamepad) {
        if (gamepad == null) {
            throw new IllegalArgumentException("Gamepad cannot be null");
        }

        gamepadList.add(gamepad);
        gamepad.setAtRest(false);
        update();
    }

    public void insertGamepad(int index, CustomGamepad gamepad) {
        if (gamepad == null) {
            throw new IllegalArgumentException("Gamepad null");
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
        } else if (index == currentIndex && currentIndex >= gamepadList.size()) {
            currentIndex = gamepadList.size() - 1;
        }

        update();
        return removed;
    }

    public void invalidateAll() {
        for (CustomGamepad gamepad : gamepadList) {
            if (gamepad != null) {
                gamepad.invalidateAll();
            }
        }
    }

    @Override
    public String toString() {
        return "ListLayeringSystem{currentIndex=" + currentIndex
                + ", totalLayers=" + gamepadList.size() + ", hasNext=" + hasNext()
                + ", hasPrevious=" + hasPrevious() + "}";
    }
}
