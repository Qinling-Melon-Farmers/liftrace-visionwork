#!/usr/bin/env bash
# git_commit_cn.sh - 用 UTF-8 消息文件做中文提交，绕开 Windows->WSL argv 编码边界。
#
# 背景：Windows 侧 pwsh 以系统 ANSI(GBK) 把参数传给 wsl.exe，WSL 内 bash 按
# UTF-8 解析，命令行内联中文会乱码/丢字。commit message 等中文内容一律先写
# UTF-8 文件，再用本脚本提交。
#
# 用法：
#   git_commit_cn.sh <消息文件> [git add 路径...]
# 消息文件允许带 BOM（Windows Set-Content -Encoding utf8 会写入 BOM），脚本自动去除。
set -euo pipefail

MSG_FILE="${1:?用法: git_commit_cn.sh <消息文件> [git add 路径...]}"
shift

if [ ! -f "${MSG_FILE}" ]; then
  echo "消息文件不存在: ${MSG_FILE}" >&2
  exit 1
fi

# 去 BOM + 保证末尾换行，统一 LF
python3 - "${MSG_FILE}" <<'PY'
import io
import sys
p = sys.argv[1]
data = io.open(p, "rb").read()
if data.startswith(b"\xef\xbb\xbf"):
    data = data[3:]
data = data.replace(b"\r\n", b"\n").rstrip(b"\n") + b"\n"
io.open(p, "wb").write(data)
PY

if [ "$#" -gt 0 ]; then
  git add "$@"
fi
git commit -F "${MSG_FILE}"
rm -f "${MSG_FILE}"
