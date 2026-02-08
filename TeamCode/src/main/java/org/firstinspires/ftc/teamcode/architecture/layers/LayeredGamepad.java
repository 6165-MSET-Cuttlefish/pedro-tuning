package org.firstinspires.ftc.teamcode.architecture.layers;

import org.firstinspires.ftc.teamcode.architecture.layers.suppliers.CustomGamepad;
import org.firstinspires.ftc.teamcode.architecture.layers.suppliers.EnhancedBooleanSupplier;
import org.firstinspires.ftc.teamcode.architecture.layers.suppliers.EnhancedDoubleSupplier;

public class LayeredGamepad<T> {
    private final LayeringSystem<T> layeringSystem;

    private final EnhancedDoubleSupplier leftStickX;
    private final EnhancedDoubleSupplier leftStickY;
    private final EnhancedDoubleSupplier rightStickX;
    private final EnhancedDoubleSupplier rightStickY;
    private final EnhancedDoubleSupplier leftTrigger;
    private final EnhancedDoubleSupplier rightTrigger;

    private final EnhancedBooleanSupplier a;
    private final EnhancedBooleanSupplier b;
    private final EnhancedBooleanSupplier x;
    private final EnhancedBooleanSupplier y;

    private final EnhancedBooleanSupplier dpadUp;
    private final EnhancedBooleanSupplier dpadDown;
    private final EnhancedBooleanSupplier dpadLeft;
    private final EnhancedBooleanSupplier dpadRight;

    private final EnhancedBooleanSupplier leftBumper;
    private final EnhancedBooleanSupplier rightBumper;

    private final EnhancedBooleanSupplier start;
    private final EnhancedBooleanSupplier back;
    private final EnhancedBooleanSupplier guide;

    private final EnhancedBooleanSupplier leftStickButton;
    private final EnhancedBooleanSupplier rightStickButton;

    private final EnhancedBooleanSupplier touchpad;
    private final EnhancedDoubleSupplier touchpadFinger1X;
    private final EnhancedDoubleSupplier touchpadFinger1Y;

    public LayeredGamepad(LayeringSystem<T> layeringSystem) {
        if (layeringSystem == null) {
            throw new IllegalArgumentException("Layering null");
        }
        this.layeringSystem = layeringSystem;

        this.leftStickX = new EnhancedDoubleSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null ? activeGamepad.getLeftStickX().getState() : 0.0;
        });

        this.leftStickY = new EnhancedDoubleSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null ? activeGamepad.getLeftStickY().getState() : 0.0;
        });

        this.rightStickX = new EnhancedDoubleSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null ? activeGamepad.getRightStickX().getState() : 0.0;
        });

        this.rightStickY = new EnhancedDoubleSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null ? activeGamepad.getRightStickY().getState() : 0.0;
        });

        this.leftTrigger = new EnhancedDoubleSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null ? activeGamepad.getLeftTrigger().getState() : 0.0;
        });

        this.rightTrigger = new EnhancedDoubleSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null ? activeGamepad.getRightTrigger().getState() : 0.0;
        });

        this.a = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getA().getState();
        });

        this.b = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getB().getState();
        });

        this.x = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getX().getState();
        });

        this.y = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getY().getState();
        });

        this.dpadUp = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getDpadUp().getState();
        });

        this.dpadDown = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getDpadDown().getState();
        });

        this.dpadLeft = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getDpadLeft().getState();
        });

        this.dpadRight = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getDpadRight().getState();
        });

        this.leftBumper = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getLeftBumper().getState();
        });

        this.rightBumper = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getRightBumper().getState();
        });

        this.start = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getStart().getState();
        });

        this.back = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getBack().getState();
        });

        this.guide = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getGuide().getState();
        });

        this.leftStickButton = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getLeftStickButton().getState();
        });

        this.rightStickButton = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getRightStickButton().getState();
        });

        this.touchpad = new EnhancedBooleanSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null && activeGamepad.getTouchpad().getState();
        });

        this.touchpadFinger1X = new EnhancedDoubleSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null ? activeGamepad.getTouchpadFinger1X().getState() : 0.0;
        });

        this.touchpadFinger1Y = new EnhancedDoubleSupplier(() -> {
            CustomGamepad activeGamepad = getActiveGamepad();
            return activeGamepad != null ? activeGamepad.getTouchpadFinger1Y().getState() : 0.0;
        });
    }

    public LayeringSystem<T> getLayeringSystem() {
        return layeringSystem;
    }

    public CustomGamepad getActiveGamepad() {
        return layeringSystem.getGamepad();
    }

    public T getCurrentLayer() {
        return layeringSystem.getLayer();
    }

    public void setLayer(T layer) {
        layeringSystem.setLayer(layer);
        invalidateAll();
    }

    public void update() {
        layeringSystem.update();
    }

    public EnhancedDoubleSupplier getLeftStickX() {
        return leftStickX;
    }

    public EnhancedDoubleSupplier getLeftStickY() {
        return leftStickY;
    }

    public EnhancedDoubleSupplier getRightStickX() {
        return rightStickX;
    }

    public EnhancedDoubleSupplier getRightStickY() {
        return rightStickY;
    }

    public EnhancedDoubleSupplier LT() {
        return leftTrigger;
    }

    public EnhancedDoubleSupplier RT() {
        return rightTrigger;
    }

    public EnhancedBooleanSupplier A() {
        return a;
    }

    public EnhancedBooleanSupplier B() {
        return b;
    }

    public EnhancedBooleanSupplier X() {
        return x;
    }

    public EnhancedBooleanSupplier Y() {
        return y;
    }

    public EnhancedBooleanSupplier DPAD_UP() {
        return dpadUp;
    }

    public EnhancedBooleanSupplier DPAD_DOWN() {
        return dpadDown;
    }

    public EnhancedBooleanSupplier DPAD_LEFT() {
        return dpadLeft;
    }

    public EnhancedBooleanSupplier DPAD_RIGHT() {
        return dpadRight;
    }

    public EnhancedBooleanSupplier LB() {
        return leftBumper;
    }

    public EnhancedBooleanSupplier RB() {
        return rightBumper;
    }

    public EnhancedBooleanSupplier getStart() {
        return start;
    }

    public EnhancedBooleanSupplier getBack() {
        return back;
    }

    public EnhancedBooleanSupplier getGuide() {
        return guide;
    }

    public EnhancedBooleanSupplier LSB() {
        return leftStickButton;
    }

    public EnhancedBooleanSupplier RSB() {
        return rightStickButton;
    }

    public EnhancedBooleanSupplier getTouchpad() {
        return touchpad;
    }

    public EnhancedDoubleSupplier TX() {
        return touchpadFinger1X;
    }

    public EnhancedDoubleSupplier TY() {
        return touchpadFinger1Y;
    }

    public EnhancedBooleanSupplier C() {
        return A();
    }

    public EnhancedBooleanSupplier O() {
        return B();
    }

    public EnhancedBooleanSupplier Q() {
        return X();
    }

    public EnhancedBooleanSupplier T() {
        return Y();
    }

    public EnhancedBooleanSupplier getOptions() {
        return getStart();
    }

    public void invalidateAll() {
        leftStickX.invalidate();
        leftStickY.invalidate();
        rightStickX.invalidate();
        rightStickY.invalidate();
        leftTrigger.invalidate();
        rightTrigger.invalidate();

        a.invalidate();
        b.invalidate();
        x.invalidate();
        y.invalidate();

        dpadUp.invalidate();
        dpadDown.invalidate();
        dpadLeft.invalidate();
        dpadRight.invalidate();

        leftBumper.invalidate();
        rightBumper.invalidate();

        start.invalidate();
        back.invalidate();
        guide.invalidate();

        leftStickButton.invalidate();
        rightStickButton.invalidate();

        touchpad.invalidate();
        touchpadFinger1X.invalidate();
        touchpadFinger1Y.invalidate();

        if (layeringSystem instanceof MapLayeringSystem) {
            ((MapLayeringSystem<?>) layeringSystem).invalidateAll();
        } else if (layeringSystem instanceof ListLayeringSystem) {
            ((ListLayeringSystem) layeringSystem).invalidateAll();
        } else if (layeringSystem instanceof CircularLayeringSystem) {
            ((CircularLayeringSystem) layeringSystem).invalidateAll();
        }
    }

    public boolean isActive(CustomGamepad gamepad) {
        return layeringSystem.isActive(gamepad);
    }

    @Override
    public String toString() {
        return "";
    }
}
