# Source revisions

Navigation: Qinling-Melon-Farmers/liftrace-controlwork, branch `feat/vcl06-local-full-mission`, commit `fbaf58b4630ffa21c810636a938a3588d68f0550`.

Vision and simulation assets: Qinling-Melon-Farmers/liftrace-visionwork, branch `feat/vdeploy-final-closeout-plan`, commit `e2214ec2a3dcece9a06bc34803628e73612be274`.

The competition checkout imports the active navigation packages from the navigation branch, retaining their upstream source and licenses. Mechanical actuator implementation and frozen historical/reference copies remain in the source repositories and branches. Servo.srv and the mock endpoint remain in the competition checkout because they are active integration interfaces.

Competition-only packaging adds the hardware application entry, shared planner configuration and local build/run scripts. It removes inactive visual/search executables and unused fixed target waypoints; source development branches retain the original packages and protected snapshots.
