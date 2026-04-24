package org.firstinspires.ftc.teamcode.math;

public class ShooterEquation {
    public double getTargetRPM(double distance) {
        return 0.00000133333 * Math.pow(distance, 4)
                - 0.000666667 * Math.pow(distance, 3)
                + 0.0766667 * Math.pow(distance, 2)
                + 21.66667 * distance
                + 600;
    }

    public double getHoodPos(double distance) {
        return 1;
    }

    public double getAirTime(double distance) {
        return ((6.66667e-8) * Math.pow(distance, 3) - 0.00003 * Math.pow(distance, 2) + 0.00633333 * distance - 0.1);
    }
}
