"""Independent, no-target single-release command contract. No I/O or ROS calls."""
FOLDER = "11_low_release_test"


def is_group(group):
    return (group.get("folder") == FOLDER and group.get("channel") == "module"
            and group.get("low_release_test") is True)


def build_command(config, group, mode, route=None, real_release=None,
                  check_config=False, slot=None, **options):
    if not is_group(group) or (route or group.get("channel")) != "module":
        raise ValueError("无靶单次释放仅允许独立模块11入口")
    if config.get("simulation", {}).get("enabled") or group.get("simulation"):
        raise ValueError("独立低空释放不支持仿真入口")
    if mode not in ("preview", "flight"):
        raise ValueError("mode 必须是 preview 或 flight")
    if not isinstance(check_config, bool) or (real_release is not None and not isinstance(real_release, bool)):
        raise ValueError("check_config / real_release 必须是布尔值")
    slot = group.get("default_slot", 1) if slot is None else slot
    if type(slot) is not int or slot not in (1, 2, 3):
        raise ValueError("slot 只允许后仓1、右仓2、左仓3")
    for key, value in options.items():
        if key in ("motion_optimized", "coverage_shiftback"):
            invalid = value is not False
        else:
            invalid = value is not None
        if invalid:
            raise ValueError("独立低空释放不接受专项覆盖参数：%s" % key)
    # preview/check must never carry release permission, even with real selected.
    command = "bash deployment/board_trials_4x4/%s/start.sh %s --slot %d" % (
        FOLDER, "preview" if check_config else mode, slot)
    if mode == "flight" and not check_config and real_release is True:
        command += " --real-release"
    if check_config:
        command += " --check-config"
    return command, ("独立无靶单次释放：FC离地0.60m前移1m，悬停5s，下降目标0.45m，"
                     "低于0.50m仅释放所选仓一次，随后悬停等待人工接管。"
                     "默认mock；preview/check仅展开配置。READY后飞手人工解锁并重新拨入OFFBOARD，"
                     "到FC离地0.60m后自动开始动作，无需另点启动任务。"
                     "真实释放复用已有5b服务，不自动启动真实PWM、不复位。")
