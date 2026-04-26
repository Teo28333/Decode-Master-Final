package org.firstinspires.ftc.teamcode.subsystems;

public interface SubsystemInterface {

    /** Read all sensor inputs into local state. */
    void read();

    /** Write all outputs to hardware. */
    void write();

    /** Call read(), write(), and telemetry() in order. */
    void update();

    /** Push subsystem state to telemetry. */
    void telemetry();
}
