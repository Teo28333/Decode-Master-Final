package org.firstinspires.ftc.teamcode.math;

import static org.firstinspires.ftc.teamcode.subsystems.constant.ShooterConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Velocity controller with motion profiling, PIDF, and voltage compensation.
 * Tuning order: kF → kS → kP → kD → kI → kA
 */
public class SpeedController {

    private static final double MAX_ACCEL            = 6000.0;
    private static final double VELOCITY_ALPHA       = 0.7;
    private static final double KS_DEADBAND          = 50.0;
    private static final double INTEGRAL_POWER_LIMIT = 1.0;

    private final VoltageSensor voltageSensor;

    private double integral        = 0.0;
    private double lastError       = 0.0;
    private double filteredVel     = 0.0;
    private double lastFilteredVel = 0.0;
    private double profiledTarget  = 0.0;
    private long   lastTimeNs      = -1L;

    public SpeedController(HardwareMap hardwareMap) {
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public double calculate(double targetVel, double measuredVel) {
        long nowNs = System.nanoTime();

        if (lastTimeNs < 0) {
            lastTimeNs      = nowNs;
            filteredVel     = measuredVel;
            lastFilteredVel = measuredVel;
            profiledTarget  = measuredVel;
            return 0.0;
        }

        double dt = (nowNs - lastTimeNs) * 1e-9;
        lastTimeNs = nowNs;
        if (dt <= 0) return 0.0;

        lastFilteredVel = filteredVel;
        filteredVel = VELOCITY_ALPHA * filteredVel + (1.0 - VELOCITY_ALPHA) * measuredVel;

        double maxDelta              = MAX_ACCEL * dt;
        double previousProfiledTarget = profiledTarget;
        profiledTarget += clamp(targetVel - profiledTarget, -maxDelta, maxDelta);
        double profiledAccel = (profiledTarget - previousProfiledTarget) / dt;

        double error      = profiledTarget - filteredVel;
        double derivative = -(filteredVel - lastFilteredVel) / dt;

        if (Math.abs(kP * error) < INTEGRAL_POWER_LIMIT) {
            integral = clamp(integral + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
        }

        double feedforward = kF * profiledTarget;
        double staticTerm  = Math.abs(profiledTarget) > KS_DEADBAND
                           ? Math.copySign(kS, profiledTarget)
                           : 0.0;

        double feedback = kP * error + kI * integral + kD * derivative;

        double voltage      = voltageSensor.getVoltage();
        double voltageScale = voltage > 0.1 ? nominalVoltage / voltage : 1.0;

        double output = (feedforward + staticTerm + feedback) * voltageScale;

        lastError = error;
        return clamp(output, -1.0, 1.0);
    }

    public void reset() {
        integral        = 0.0;
        lastError       = 0.0;
        filteredVel     = 0.0;
        lastFilteredVel = 0.0;
        profiledTarget  = 0.0;
        lastTimeNs      = -1L;
    }

    public double getFilteredVelocity() { return filteredVel;    }
    public double getProfiledTarget()   { return profiledTarget; }
    public double getError()            { return lastError;      }
    public double getIntegral()         { return integral;       }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
