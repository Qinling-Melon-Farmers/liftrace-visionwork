# WSL 8771 双工程选择（2026-10-08）

当前先准备今晚08，部署构建尚未完成。只保存本地连接参数、读取本地快照；板端连接、HTTP命令生成和preview均等待主代理明确通知部署构建完成。本轮禁止真实舵机、飞行、ready或设备/任务启动。

## 工程与卡片必须成套选择

| 用途 | board_root | env_script | site_dir | 卡片/配置 |
| --- | --- | --- | --- | --- |
| 今晚08先测试 | `/home/orangepi/liftrace_board_trials_20260928` | `deployment/site_20260928/environment.sh` | `deployment/site_20260928` | `mod08`；`deployment/board_trials_4x4/08_full_mission/site_20261007_221730.yaml`；显式选择速度 `competition` |
| 独立正赛 | `/home/orangepi/liftrace_competition_20261008` | `deployment/competition/environment.sh` | `deployment/competition` | `competition`；在该工程中明确选择所需正赛YAML |

两套host均为 `orangepi@192.168.3.126`、port为22。08的competition速度只覆盖速度/前视等参数，保留221730现场几何和FC搜索2m；它不切换根目录，也不等于独立正赛。该档请求应含 `speed_profile: "competition"`，预期入口参数含 `--speed-profile competition`，规划上限1.2m/s、1.0m/s²。未选择速度时仍继承有限空间默认，不能认为已是competition。

现有网页只持久化所选卡片ID，不保存速度选项。打开/刷新页面后先点08卡“选择此组”，再显式选competition速度，每次生成前复核。浏览器原先记住的卡片可能仍是正赛；本次不替用户改浏览器状态。

## 使用现有保存机制

连接设置“保存”对应 `POST /api/config`。必须在同一个请求中提交host/root/env/site_dir。包含host会把整套当前连接参数写入WSL仓库外 `~/.config/liftrace-flight-workbench/profile.json`（0600），下次启动覆盖workbench.yaml默认；仅改root/env通常只改服务内存。保存不连接SSH、不运行探针或节点。独立 `GET /api/config` 不是读取接口，读取用 `GET /api/snapshot`。

今晚08保存请求（已在本地profile准备，尚未连接）：

```json
{
  "host": "orangepi@192.168.3.126",
  "port": 22,
  "board_root": "/home/orangepi/liftrace_board_trials_20260928",
  "site_dir": "deployment/site_20260928",
  "env_script": "deployment/site_20260928/environment.sh"
}
```

独立正赛保存请求（待切换时使用，本轮不提交）：

```json
{
  "host": "orangepi@192.168.3.126",
  "port": 22,
  "board_root": "/home/orangepi/liftrace_competition_20261008",
  "site_dir": "deployment/competition",
  "env_script": "deployment/competition/environment.sh"
}
```

model/metadata也属于全局连接设置。本轮保留现有08值；独立工程须按主代理实际部署清单核对其工程内的模型与metadata，再成套保存，不凭历史打包说明猜测文件名。配置文件均优先使用所属工程内相对路径，不用另一个工程的绝对路径。

## 防止串源

所有卡片共用全局root/env，选择 `competition` 卡片不会自动切根。0928根下即使也有 `deployment/competition/start.sh`，在它上面点正赛卡仍会执行0928工程，不能称为独立正赛。此次复用现有工作台，不新增工程选择器或后台自动切换。

1. 当前先停留在0928配置，选08并显式选competition速度。
2. 切独立正赛前须结束本工作台会话和录包/日志操作；已有会话运行时服务会拒绝修改连接参数。部署未完成时不连接、不执行收尾命令。
3. 使用上面的独立正赛请求保存成套参数，然后读取 `/api/snapshot`，确认root/env均属于独立工程，再选择正赛卡片。返回08也必须保存整套0928参数。
4. `/api/trial/command`仅返回入口命令body，不包含外层root。核对时必须结合快照的root/env；实际会话包装为 `cd <board_root> && source <board_root>/<env_script> && <body>`。不能只看body就认为已切源。
5. 独立正赛environment会检查uav_mission/uav_vision/uav_high_view/camera_sdk/actuator_pwm的rospack路径必须落在独立根内。0928environment没有相同逐包守卫；其板端构建/overlay来源由主代理完成后检查。本轮不source任何板端环境。
6. 现有雷达终端仍固定引用当前根下 `deployment/site_20260928/MID360_config.json`，通用preflight仍检查site_dir下的test_area.yaml。独立工程是否带这些设备配置需要按部署清单核对；正赛根设置site_dir=deployment/competition后，不能把通用test_area缺项误判成正赛场地配置缺项，也不能为消除告警借用0928工程的文件。本轮不启动这些终端。

## 待构建完成的调用清单（未执行）

08纯命令生成：`POST /api/trial/command`：

```json
{"group_id":"mod08","mode":"preview","route":"module","real_release":false,"check_config":true,"speed_profile":"competition"}
```

核对08入口、221730现场YAML、competition速度和当前0928快照成套一致。若再执行preview，使用相同请求发 `/api/trial/start`，仅mode=preview/check_config=true；该接口会开板端会话，须等部署完成后再调用，不因名称含preview而提前执行。

独立正赛纯命令生成：切好独立根并检查快照后，`POST /api/trial/command`：

```json
{"group_id":"competition","mode":"preview","real_release":false,"check_config":true,"competition_config":"deployment/competition/field.example.yaml"}
```

正式模板尚未实测确认时，只能检查展开结果，不能宣称可生成飞行配置。需要生成runtime时使用主代理确认的实测正赛YAML与参考位姿；模板本身不能代替这些数据。后续preview检查结束恢复0928配置，用户仍先测试08。

本轮不调用 `/api/connect`、`/api/action/preflight`、`/api/action/start_all`、`/api/session/open`、`/api/trial/start` 或 `/api/action/mission_start`，不操作PWM/舵机/飞控。后续HTTP生成/preview本身不构成真实飞行授权。
