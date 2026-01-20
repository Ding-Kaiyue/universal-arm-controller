---
name: 安装问题
about: 报告安装过程中遇到的问题
title: '[INSTALLATION] '
labels: ['installation', 'help wanted']
assignees: ['Ding-Kaiyue']

---

## 安装方式
- [ ] Docker（从 Docker Hub 拉取）
- [ ] Docker（从本地镜像加载）
- [ ] 本地编译
- [ ] 其他: ___________

## 环境信息
- **操作系统**: [例如 Ubuntu 22.04 LTS]
- **CPU 架构**: [例如 x86_64, ARM64]
- **内存**: [例如 16GB RAM + 8GB Swap]
- **ROS 版本**: [例如 ROS2 Humble]
- **编译器版本**: [例如 gcc 11.3.0]

## 安装步骤
请详细描述你执行的安装步骤：

```bash
# 请粘贴你执行的命令
```

## 错误信息
请粘贴完整的错误信息或日志：

```
# 错误日志
```

## 已尝试的解决方案
- [ ] 检查了 [安装指南](https://github.com/Ding-Kaiyue/universal-arm-controller/blob/develop/docs/getting_started/INSTALLATION.md)
- [ ] 检查了 [故障排除指南](https://github.com/Ding-Kaiyue/universal-arm-controller/blob/develop/docs/getting_started/TROUBLESHOOTING.md)
- [ ] 清理了编译缓存（`rm -rf build install log`）
- [ ] 重新安装了依赖

请描述你已经尝试过的其他解决方案：

## 期望结果
描述安装成功后应该看到的结果：

## 附加信息
任何其他相关信息，如：
- 网络环境（是否需要代理）
- 特殊的系统配置
- 相关的错误日志文件

---

**提示**:
- 如果是 Docker 相关问题，请提供 `docker version` 和 `docker info` 的输出
- 如果是编译问题，请提供完整的编译日志（可以使用 `colcon build 2>&1 | tee build.log`）
- 如果是依赖问题，请提供 `apt list --installed | grep -E "ros|eigen|osqp|pinocchio"` 的输出
