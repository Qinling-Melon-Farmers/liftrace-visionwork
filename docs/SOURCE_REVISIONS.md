# Source revisions

Navigation: Qinling-Melon-Farmers/liftrace-controlwork, branch `feat/vcl06-local-full-mission`, commit `987b57c6c28a5bbc2a793c0d38222f7daf061ebb`.

Vision and simulation assets: Qinling-Melon-Farmers/liftrace-visionwork, branch `feat/vdeploy-final-closeout-plan`, commit `1b328c13ab4d302d5f69809df6ebc24f6cb1cd81`.

The competition checkout imports the active navigation packages from the navigation branch, retaining their upstream source and licenses. Mechanical actuator implementation and frozen historical/reference copies remain in the source repositories and branches. Servo.srv and the mock endpoint remain in the competition checkout because they are active integration interfaces.

Competition-only packaging adds the hardware application entry, shared planner configuration and local build/run scripts. It removes inactive visual/search executables and unused fixed target waypoints; source development branches retain the original packages and protected snapshots.

Main integration retains original assets outside the active competition build. Its explicit package list matches the clean competition checkout. AGENTS.md retains the main repository workflow; the clean branch has its own directory-specific instructions.
