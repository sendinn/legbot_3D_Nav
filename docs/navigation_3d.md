# 在 RViz 手动标定三维目标

先退出已有 simulation.sh / mapping.sh，然后运行：

```bash
./simulation.sh --3d
```

机械狗会起身并等待，启动时不会自动执行预设目标。

1. 点击工具栏 **3D Nav Goal**（快捷键 G）。
2. 在目标楼层的可通行点云表面按下左键，拖动箭头方向。
3. 松开左键，即发送三维位置并开始规划，不需要右键菜单。
4. 再次使用 3D Nav Goal 可替换目标。按 Esc 或右键取消当前尚未发送的选择。

Z 直接取点击表面的真实三维高度，不固定在地面 Z=0。
Tool Properties 中的 Height offset 可以在表面高度上增加偏移，默认 0。
当前导航以目标位置/楼层为到达条件，拖动方向不作为最终朝向验收条件。

**3D Pose Estimate**（快捷键 I）发布 /initialpose 初始位姿，作用是定位初始化，不是导航。
当前 Gazebo 真值定位没有该接收端，因此工具明确提示不需要初始化，且不发送估计。
它不会移动仿真机器人或改变 Gazebo 真值。

橙色交互目标球保留为目标预览及规划状态显示，也可继续使用旧的拖动与右键执行方式。

使用旧的目标球时，拖动只修改候选目标；工具栏 3D Nav Goal 则在松开鼠标时发送。无路径、地图外目标或定位未就绪时会在终端日志中报告错误，不会发布该目标。
目标规划失败时不会替换已经执行中的旧任务。Ctrl+C 停止本次仿真。
Publish Point 点击的是三维点云表面；Z 选择楼层，最终行走参考会按可通行地面加机身高度生成，并非飞往悬空坐标。

当前交互模式基于自带的 Building / building2_9 地图，使用 Gazebo 真值定位。
building_pct 坐标到 Gazebo 世界的偏移为 X+13 米。无需手算：RViz 点选会自动转换坐标。
机器人在西侧地图外出生区时，先通过既有地面入口接入楼栋 PCT 路线；进入楼栋后从当前位置规划。
自己保存的 PCD 尚未自动替换预建地图。

日志位于 log/simulation/<时间戳-PID>/crossfloor.log。
出现 Interactive goal executed 和 PCT cross-floor mission started 表示已发送任务。

旧的自动目标入口仍可用：
`./simulation.sh --upstairs` 或 `./simulation.sh --goal3d 2 -3 4.5`。
手动标定请使用 `--3d`。

验证包含：启动无自动路径、点选坐标转换、拖动不执行、菜单执行、从更新位置重新规划、连续任务转发和地图外目标拒绝。
未完成所有目标的全程上楼到达验收。

如果看不到橙色目标球：展开左侧 Displays 的 “3D goal - right click to execute”，
确认启用，并将 “Interactive Markers Namespace” 设为 /basic_controls。
ROS 2 Humble 此字段不是 Update Topic。当前窗口可直接改，无需重启机器人。

## 视角与实时雷达

默认视角为 Scene overview，包含机械狗出生区和楼栋。
Views 中保存了 Follow GO2（跟随机械狗）和 Building - select floor（楼栋选点）。
实时雷达位于 Robot sensing and state / SCAN obstacle/elevation cloud，
话题 /livox/points_world，Reliability Policy 必须为 Best Effort。
预建 PCT 地图只覆盖楼栋；周围障碍区显示的是实时雷达观测，不等于整张世界静态地图。
