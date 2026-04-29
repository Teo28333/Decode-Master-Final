package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.util.ElapsedTime;

class AutoStateMachine<T extends Enum<T>> {
    private final ElapsedTime stepTimer = new ElapsedTime();
    private T step;

    AutoStateMachine(T initialStep) {
        step = initialStep;
        stepTimer.reset();
    }

    T getStep() {
        return step;
    }

    void setStep(T nextStep) {
        step = nextStep;
        stepTimer.reset();
    }

    double elapsedMs() {
        return stepTimer.milliseconds();
    }
}
