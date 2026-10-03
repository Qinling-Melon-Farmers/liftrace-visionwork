# 2026-09-27 板端旧控制补丁前快照

按 AGENTS 规则14保存本分支修改前 patrol_control 已纳管源码包、文件清单和 SHA256。不参与编译。之后仅继承视觉研究分支 c9df53f 的 near_wall_align.h、patrol_control.cpp 边界夹取/释放检查、对应测试；不重写旧状态机。现场相机、槽位、帧转换和执行器参数保持由配置注入。
