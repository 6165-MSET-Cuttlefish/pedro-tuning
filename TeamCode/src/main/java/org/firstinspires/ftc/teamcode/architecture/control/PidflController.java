package org.firstinspires.ftc.teamcode.architecture.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PidflController {
    public double p = 0, i = 0, d = 0, f = 0, l = 0;
    public double integralMax = Double.POSITIVE_INFINITY;
    private double previousError;
    private double error;
    private double integralSum;
    private double position;
    private double target;
    private double errorDerivative;
    private final ElapsedTime timer;

    public PidflController() {
        timer = new ElapsedTime();
    }

    public double getPidfl() {
        double proportional = error * p;
        double integral = integralSum * i;
        double derivative = errorDerivative * d;
        double feedforward = target * f;
        double lowerLimit = Math.signum(error) * l;

        return proportional + integral + derivative + feedforward + lowerLimit;
    }

    public void updatePosition(double update) {
        position = update;
        previousError = error;
        error = target - position;

        double deltaTimeSeconds = timer.seconds();
        timer.reset();

        if (deltaTimeSeconds > 0) {
            errorDerivative = (error - previousError) / deltaTimeSeconds;
        } else {
            errorDerivative = 0;
        }

        integralSum += error * deltaTimeSeconds;

        if (Math.abs(integralSum) > integralMax) {
            integralSum = Math.signum(integralSum) * integralMax;
        }
    }

    public void update(double target, double current) {
        setTarget(target);
        updatePosition(current);
    }

    public void reset() {
        previousError = 0;
        error = 0;
        integralSum = 0;
        position = 0;
        target = 0;
        errorDerivative = 0;
        timer.reset();
    }

    public void setTarget(double set) {
        target = set;
        integralSum = 0;
    }

    public double getError() {
        return error;
    }

    public double getIntegralSum() {
        return integralSum;
    }

    public double getErrorDerivative() {
        return errorDerivative;
    }

    public void setController(double p, double i, double d, double f, double l) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.l = l;
    }

    public void setIntegralMax(double integralMax) {
        this.integralMax = integralMax;
    }

    public boolean isAtTarget(double tolerance) {
        return Math.abs(error) <= tolerance;
    }
}
