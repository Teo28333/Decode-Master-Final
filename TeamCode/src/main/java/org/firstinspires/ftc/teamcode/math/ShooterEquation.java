package org.firstinspires.ftc.teamcode.math;

/**
 * Maps distance (inches) to shooter RPM, hood position, and ball air time.
 *
 * Replace getTargetRPM(), getHoodPos(), and getAirTime() with your
 * measured/fitted data curves. Use a spreadsheet to fit a polynomial
 * or piecewise linear curve to your measured data points.
 */
public class ShooterEquation {

    /**
     * Returns the target flywheel RPM for a given distance.
     * @param distanceInches distance from turret pivot to goal (inches)
     */
    public double getTargetRPM(double distanceInches) {
        // TODO: replace with fitted curve from field measurements
        return 2000 + distanceInches * 10;
    }

    /**
     * Returns the hood servo position [0, 1] for a given distance.
     * @param distanceInches distance from turret pivot to goal (inches)
     */
    public double getHoodPos(double distanceInches) {
        // TODO: replace with fitted curve from field measurements
        double pos = 0.3 + (distanceInches / 144.0) * 0.4;
        return Math.max(0.0, Math.min(1.0, pos));
    }

    /**
     * Returns estimated ball air time in seconds for a given distance.
     * Used for shoot-on-the-move goal compensation in TurretSS and ShooterSS.
     * @param distanceInches distance from turret pivot to goal (inches)
     */
    public double getAirTime(double distanceInches) {
        // TODO: refine with projectile motion or measured values
        return 0.1 + distanceInches * 0.001;
    }
}
