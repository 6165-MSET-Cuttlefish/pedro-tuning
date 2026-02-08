package org.firstinspires.ftc.teamcode.architecture.layers;

import org.firstinspires.ftc.teamcode.architecture.layers.suppliers.CustomGamepad;

public interface LayeringSystem<T> {
    void setLayer(T layer);

    T getLayer();

    CustomGamepad getGamepad();

    boolean isActive(CustomGamepad gamepad);

    void update();
}
