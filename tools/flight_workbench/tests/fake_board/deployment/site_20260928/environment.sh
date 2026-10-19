#!/usr/bin/env bash
# 现场环境脚本（纸板自检用）：只把 fake shim 加进 PYTHONPATH，不启动任何节点。
fake_root="$(cd "${BASH_SOURCE[0]%/*}/../.." && pwd)"
export PYTHONPATH="$fake_root/shims:${PYTHONPATH:-}"
