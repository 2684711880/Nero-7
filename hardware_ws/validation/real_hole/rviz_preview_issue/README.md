# RViz 旧起点预览诊断

`before.json` 为共享预览话题抓包，正确任务起点（约 0.006° 偏差）被后来的旧 park 起点（最大约 184.535° 偏差）覆盖。实时反馈和 MoveIt 当前状态核对一致。

`after.json` 为独立预览话题验证，记录起点误差 0°。这些是消息层检查，不是实物定位误差。预览桥的回归测试另验证：有效 RViz Plan 转发，随后 20 条旧起点消息均不覆盖正确预览。

实现见 `scripts/real_hole_preview.py`、`scripts/real_hole_preview_bridge.py`；不包含硬件命令。
