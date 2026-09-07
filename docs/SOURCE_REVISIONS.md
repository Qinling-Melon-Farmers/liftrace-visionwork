# Source revisions

当前 R56 实跑源：精简整机 f0f748016064fbecabefd03d7b05503fb744a697；导航修复 4312488b66506e33464fde8a39a962428adff630；main 集成准备 548dffcde60c626f4f7d906be94c7c562dc06932。后续文档/诊断工具提交不是飞行源码。main 仍为 44359e8ba426ed91ef42c4951b5ab2fae8924027，未合并。

下面两项是精简导入起点，仅作历史来源，不是当前运行 HEAD。

Navigation: Qinling-Melon-Farmers/liftrace-controlwork, branch `feat/vcl06-local-full-mission`, commit `987b57c6c28a5bbc2a793c0d38222f7daf061ebb`.

Vision and simulation assets: Qinling-Melon-Farmers/liftrace-visionwork, branch `feat/vdeploy-final-closeout-plan`, commit `1b328c13ab4d302d5f69809df6ebc24f6cb1cd81`.

The competition checkout imports the active navigation packages from the navigation branch, retaining their upstream source and licenses. Mechanical actuator implementation and frozen historical/reference copies remain in the source repositories and branches. Servo.srv and the mock endpoint remain in the competition checkout because they are active integration interfaces.

Competition-only packaging adds the hardware application entry, shared planner configuration and local build/run scripts. It removes inactive visual/search executables and unused fixed target waypoints; source development branches retain the original packages and protected snapshots.
