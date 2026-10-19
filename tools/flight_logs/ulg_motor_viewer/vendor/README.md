# 随包解析器

仅复制本机既有 `rl_drone` 中 pyulog **1.2.4** 的 `__init__.py`、`core.py`、`px4.py` 与原 BSD-3-Clause `LICENSE.md`，源码未修改。来源项目为 PX4/pyulog，版权及许可见随附文件。只包含读取 ULog 所需的纯 Python 子集，不包含命令行工具或数据库功能。

优先使用当前环境已安装的 pyulog；缺少时使用此副本。仍需已有 numpy 与 matplotlib，不安装或打包 Python 环境。这样 Windows 原生环境无需为 pyulog 单独安装包；本机/板端 profile、SSH设置及日志均不在此目录。
