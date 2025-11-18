# 贡献指南 (Contributing Guide)

感谢你对本项目的关注！本指南将帮助你了解如何有效地为项目做出贡献。

## 欢迎贡献！

我们欢迎所有形式的贡献，包括但不限于：

### 贡献类型

✅ **欢迎的贡献：**
- 🐛 Bug 修复
- ✨ 新功能开发
- 📚 文档改进
- 🧪 测试用例
- 🚀 性能优化
- 💡 功能建议

---

## 🌳 分支策略

### 主要分支
- **master**: 生产就绪代码，每个提交都应该是稳定版本
- **develop**: 开发分支，集成新功能的地方
- **feature/\***: 功能分支，从 develop checkout
- **bugfix/\***: 修复分支，从 develop checkout
- **hotfix/\***: 紧急修复，从 master checkout 后也merge回master和develop

### 分支保护规则 🔒
- `master` 和 `develop` 受保护，不允许直接push
- 所有改动必须通过 Pull Request
- PR 需要至少 2 个核准（LGTM）
- 必须通过 CI/CD 流水线
- 代码覆盖率必须 ≥ 95%
- 所有代码风格检查必须通过
- 禁止 force push 到受保护分支

---

## 🔄 标准工作流程

### 第1步：Fork 仓库（如果是外部贡献者）

```bash
# 在 GitHub 上 fork 本仓库
git clone https://github.com/Ding-Kaiyue/universal-arm-controller.git
cd universal-arm-controller
git remote add upstream https://github.com/Ding-Kaiyue/universal-arm-controller.git
```

### 第2步：创建功能分支

```bash
# 确保本地 develop 是最新的
git fetch upstream develop
git checkout -b feature/your-feature-name upstream/develop

# 或者修复分支
git checkout -b bugfix/bug-name upstream/develop

# 或者紧急修复（仅限 hotfix）
git checkout -b hotfix/critical-issue upstream/main
```

**分支命名规范：**
- `feature/add-async-trajectory-execution` - 新功能
- `bugfix/fix-mode-switching-crash` - 缺陷修复
- `hotfix/fix-critical-safety-issue` - 紧急修复

### 第3步：开发代码

```bash
# 进行开发
git add .
git commit -m "feat: add your feature description"
```

### 第4步：编写测试

**测试覆盖率要求：≥ 95%**

```bash
# 为新功能编写单元测试
# 为关键路径编写集成测试
# 运行测试
colcon test

# 检查覆盖率
colcon test --coverage
```

### 第5步：代码风格检查

```bash
# 检查 C++ 代码风格
clang-format -i src/**/*.cpp src/**/*.hpp

# 运行静态分析
cppcheck src/

# 确保所有检查通过
```

### 第6步：提交 Pull Request

1. **Push 到你的 fork**
   ```bash
   git push origin feature/your-feature-name
   ```

2. **在 GitHub 上创建 PR**
   - Base branch: `develop` (普通功能) 或 `master` (hotfix)
   - Head branch: `YOUR_USERNAME:feature/your-feature-name`

3. **PR 描述应包含：**
   - 变更说明
   - 实现思路和设计决策
   - 关键代码片段（如果有）
   - 测试结果
   - 关联 Issue（如 Fixes #123）
   - @至少两个评审者

### 第7步：Code Review

1. **等待 CI/CD 通过**
   - 所有自动化检查必须通过
   - 代码覆盖率必须达到要求

2. **回应评审意见**
   - 解决所有评论线程
   - 每次更改后 push 新的 commit
   - 不要使用 force push

3. **获得核准**
   - 需要至少 2 个 LGTM (Looks Good To Me)
   - 维护者必须是其中之一

### 第8步：合并

仅当满足以下条件时，维护者才会 Squash and Merge：

- ✅ 所有对话已解决
- ✅ CI/CD 流水线通过
- ✅ 至少 2 个核准 (LGTM)
- ✅ 代码覆盖率 ≥ 95%
- ✅ 通过所有代码风格检查

---

## 📝 Commit Message 规范 (Conventional Commits)

```
<type>(<scope>): <subject>

<body>

<footer>
```

**类型 (type):**
- `feat`: 新功能
- `fix`: 缺陷修复
- `refactor`: 代码重构
- `perf`: 性能优化
- `test`: 添加或修改测试
- `docs`: 文档更新
- `chore`: 构建或工具链变更
- `ci`: CI/CD 配置变更

**作用域 (scope):**
- `arm_controller`: ARM控制器
- `hardware_driver`: 硬件驱动
- `trajectory_planning`: 轨迹规划
- 等等...

**示例：**
```
feat(hardware_driver): implement async trajectory execution

- Replace blocking executeTrajectory with async thread
- Add cancellation mechanism via should_stop flag
- Improve ROS2 callback responsiveness

Fixes #123
```

---

## 📋 提交前检查清单

- [ ] 代码遵循项目的代码规范（C++17, Google 风格指南）
- [ ] 为新功能或修改添加了单元测试
- [ ] 所有测试通过 (`colcon test`)
- [ ] 代码覆盖率达到 95% 或更高
- [ ] 通过了所有代码风格检查
- [ ] Commit message 遵循 Conventional Commits
- [ ] PR 描述清晰完整
- [ ] 没有未解决的对话线程
- [ ] 本地已与 develop 的最新版本同步

---

## 🚫 严格禁止

- ❌ 直接 push 到 `main` 或 `develop`
- ❌ Force push 到受保护分支
- ❌ 绕过 code review
- ❌ 提交代码覆盖率低于 95% 的代码
- ❌ 违反 Conventional Commits 的 commit message
- ❌ 未通过 CI/CD 就合并

---

## 💡 最佳实践

1. **保持分支最新**
   ```bash
   git fetch upstream develop
   git rebase upstream/develop
   ```

2. **编写清晰的 commit message**
   - 第一行 ≤ 50 字符
   - 空一行
   - 详细描述（可选）

3. **一个 PR 一个功能**
   - 不要在一个 PR 中混入多个不相关的功能

4. **积极参与 Review**
   - 尊重评审者的意见
   - 解释你的设计决策
   - 如有异议，讨论而非争执

---

## 📞 报告问题和建议

我们提供了标准化的 Issue 模板来帮助您更有效地沟通：

- **[🐛 报告 Bug](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=bug_report.md)**
- **[✨ 提出功能请求](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=feature_request.md)**
- **[❓ 提出使用问题](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=usage_question.md)**
- **[🔒 报告安全漏洞](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=security_report.md)**

---

## 📞 联系方式

- **Issues**: 通过 GitHub Issues
- **Pull Requests**: 通过 GitHub PR
- **商业合作**: kaiyue.ding@raysense.com
- **技术支持**: d18292819833 (微信)

---

感谢您的贡献！🎉 
