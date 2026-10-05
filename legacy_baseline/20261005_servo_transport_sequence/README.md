# 2026-10-05 旧控制最小修复前快照
来源板端专项44c3e4d1的patrol_control，修改目的：ALIGN误将ROS transport Header.seq作为稳定决策号，导致异步释放身份检查拒绝。保留当前版本可回滚，不参与编译。后续补丁使用Bridge已填写的嵌套goal.header.seq，不改变服务或消息字段。
