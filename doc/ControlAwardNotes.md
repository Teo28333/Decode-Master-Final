# Control Award Notes

## Main Control Solution

Our main control solution is an automatic shooting system that combines robot pose feedback, velocity feedback, voltage feedback, and state-machine logic so the robot only transfers game pieces when the shot is ready.

The robot uses Pedro Pathing localization to estimate its field position, heading, velocity, and angular velocity. From that feedback, the turret and shooter calculate where the goal is relative to the robot. The code also compensates for the robot moving while the game piece is in the air by estimating flight time and shifting the target point based on robot velocity.

Code evidence:

- `Robot.java`: reads Pedro pose and velocity every loop, sends them to the turret and shooter.
- `TurretSS.java`: uses field position, turret offset, robot velocity, angular velocity, filtering, and servo limits to aim.
- `ShooterSS.java`: calculates target RPM and hood position from distance, then uses velocity feedback to decide when it is ready.
- `PIDFSController.java`: closes the shooter velocity loop using encoder velocity, PID, feedforward, static friction compensation, anti-windup, and battery voltage compensation.
- `IntakeCommands.java`: transfer only runs when the shooter is ready and the robot/turret is aimed.

Short judge explanation:

"Instead of the driver guessing when to shoot, the robot checks external feedback first. Pedro gives us our pose and velocity, the shooter motor encoder gives actual RPM, and the battery voltage sensor helps keep the shooter response consistent. Our code uses that feedback to aim, set shooter speed, and only transfer when the shot is ready."

## Turret And Robot Heading Fail-Safe

We added a fail-safe for matches where the turret does not work. When the driver toggles turret fail-safe with `share`, the turret servos park at position `0`. Then pressing `Y` toggles robot heading aim. Instead of moving the turret, the robot rotates itself so the back of the robot points at the same target angle the turret would have used.

This uses the Pedro Pathing heading PIDF coefficients, so the backup aiming mode still uses closed-loop heading control instead of manual driver guessing.

Why this helps:

- If the turret servo fails, we still have a way to aim.
- The driver can keep driving translation while the robot controls heading.
- The same target calculation is reused, including robot velocity compensation.
- Transfer logic still waits until the heading error is inside the tolerance.

Short judge explanation:

"We identified that the turret was a single point of failure, so we built a software fail-safe. If the turret breaks, the turret parks and the drivetrain becomes the aiming system. It uses Pedro's heading PIDF and the same target math, so our shot process still has feedback control."

## Autonomous Dry Run Timing

All autonomous routines have a dry run mode toggled during init. Dry run follows the same paths and waits through the same timed actions, but skips the shooter, transfer, and intake mechanisms.

Why this helps reliability:

- We can test whether the autonomous routine fits inside the match time without launching game pieces.
- We can verify path timing without wearing out mechanisms.
- The dry run keeps the same transfer and intake wait times, so the timing test is closer to the real match routine.
- Telemetry displays the current step, elapsed step time, path progress, and dry-run state.

Short judge explanation:

"Dry run is part of how we validate autonomous reliability. It lets us run the full timing of the routine safely. Because dry run still waits for transfer and intake timing, it tells us whether the auto can finish in the time limit."

## Reliability Features

The control system includes several reliability choices:

- Shooter readiness gate: transfer only happens after the shooter is within the RPM threshold.
- Battery compensation: shooter power scales with measured voltage so low battery has less effect.
- Anti-windup: the shooter PID integral is limited to prevent unstable recovery.
- Velocity filtering: turret and shooter calculations filter noisy velocity feedback.
- Turret safety limit: if the target angle is outside the safe turret range, the robot is told to turn instead.
- Pose reset buttons: drivers can reset pose to known field positions if localization becomes wrong.
- Telemetry and Panels: live feedback shows pose, path, shooter error, turret angle, and readiness.
- Dry run: autonomous timing can be tested without running mechanisms.
- Turret fail-safe: robot heading can replace turret aiming if the turret is broken.

## Engineering Process

Problem:

Early shooting depended too much on driver timing and battery condition. When battery voltage dropped or the robot was moving, the same shot command could behave differently. The turret also created a risk because a servo issue could remove our ability to aim.

Design:

We broke the problem into smaller feedback loops:

- Use localization feedback to know where the robot is.
- Use velocity feedback to compensate shots while moving.
- Use motor encoder feedback to control shooter RPM.
- Use voltage feedback to compensate for battery drop.
- Use heading PIDF as a backup aiming method if the turret fails.

Testing:

We used telemetry and Panels to compare target RPM against current RPM, check shooter error, watch turret angle, and verify autonomous step timing. Dry run mode was added so we could repeat autonomous tests more often without stressing the shooter and intake.

Lessons learned:

- A shot should be controlled by measured readiness, not only by a fixed driver timing.
- Voltage compensation matters because the robot behaves differently late in a match.
- A backup control mode is valuable when a mechanical subsystem can fail.
- Dry-run testing is more useful when it keeps the same timing as the real autonomous.
- Telemetry needs to show both target and actual values, otherwise tuning becomes guessing.

## Possible Improvements To Mention

- Add a measured shot-success log so we can compare RPM error and heading error against made shots.
- Add automatic pose correction from vision or field targets.
- Tune separate PIDF values for normal turret aiming and fail-safe robot aiming.
- Add a dashboard graph for shooter recovery after each transfer.
- Add a confidence score that combines pose quality, shooter error, and heading error before transfer.

## 30 Second Summary

"Our Control Award focus is reliable automatic shooting. The robot uses pose and velocity feedback from Pedro Pathing, shooter encoder feedback, voltage feedback, and PIDF control to aim and shoot consistently. Transfer is gated by readiness, so the robot only feeds when the shooter is at speed and aimed. We also added dry-run timing for autonomous validation and a turret fail-safe where the drivetrain uses heading PIDF to aim if the turret breaks. The engineering process was about replacing driver guessing with measured feedback and adding reliability tools after we found failure points."
