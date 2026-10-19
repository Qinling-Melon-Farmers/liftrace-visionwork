#!/usr/bin/env bash
# 启动试飞验证看板（在 WSL 里运行；浏览器打开下面打印的地址）。
#
# 常用：
#   bash tools/flight_workbench/start_workbench.sh                    # 连现场板端
#   bash tools/flight_workbench/start_workbench.sh --transport local  # 本机自检模式（不连板端）
#   bash tools/flight_workbench/start_workbench.sh --port 8792 --open
set -euo pipefail
script_dir="${BASH_SOURCE[0]%/*}"
script_dir="$(cd "$script_dir" && pwd)"

# 优先沿用调用者已激活的 conda；也可显式指定 Python，不安装任何包。
python_bin="${WORKBENCH_PYTHON:-${CONDA_PREFIX:+${CONDA_PREFIX}/bin/python}}"
python_bin="${python_bin:-python3}"
# 本机 rl_drone 缺 pexpect 时复用已安装的纯 Python 系统包。
if ! "$python_bin" -c 'import pexpect, yaml' >/dev/null 2>&1 &&
   [[ -d /usr/lib/python3/dist-packages ]]; then
  export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}/usr/lib/python3/dist-packages"
fi
if ! "$python_bin" -c 'import pexpect, yaml' >/dev/null 2>&1; then
  echo "缺少依赖：$python_bin 需要 pexpect 与 pyyaml（本机应已随 ROS 安装）" >&2
  exit 2
fi

exec "$python_bin" "$script_dir/server.py" "$@"
