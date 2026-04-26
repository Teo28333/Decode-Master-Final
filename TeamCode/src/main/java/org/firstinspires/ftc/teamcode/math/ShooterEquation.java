package org.firstinspires.ftc.teamcode.math;

public class ShooterEquation {
    public double getTargetRPM(double distance) {
        return  -0.0000114126 * Math.pow(distance, 4)
                + 0.00518183   * Math.pow(distance, 3)
                - 0.846593     * Math.pow(distance, 2)
                + 73.04495     * distance
                + 675.72007;
    }

    public double getHoodPos(double distance) {
        return -(8.2745e-9) * Math.pow(distance, 4)
                + 0.00000330296 * Math.pow(distance, 3)
                - 0.000449633   * Math.pow(distance, 2)
                + 0.028326      * distance
                - 0.1213;
    }

    public double getAirTime(double distance) {
        return 4 * ((6.66667e-8) * Math.pow(distance, 3) - 0.00003 * Math.pow(distance, 2) + 0.00633333 * distance - 0.1);
    }
}
