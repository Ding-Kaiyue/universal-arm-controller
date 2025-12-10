# 贡献指南

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

### 贡献流程

<<<<<<< HEAD
1. **Fork 项目**
=======
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

### 第4步：编写和运行单元测试

**单元测试覆盖率要求：≥ 95%**

```bash
# 自动编译所有包并运行所有单元测试
./run_tests.sh
```

这个脚本会自动：
1. ✅ 编译所有包（启用测试）
2. ✅ 运行所有单元测试
3. ✅ 编译或测试失败时立即停止并报告错误
4. ✅ 通过后显示提交前的检查清单

#### 4.2 检查代码覆盖率

```bash
# 生成详细的覆盖率报告
./coverage_report.sh
```

这个脚本会：
1. 清除之前的覆盖率数据
2. 编译所有包（启用覆盖率统计）
3. 运行所有单元测试
4. 生成 HTML 覆盖率报告（如果安装了 lcov）

**安装覆盖率工具（可选，用于生成详细HTML报告）：**
```bash
sudo apt-get install lcov
```

**覆盖率报告位置：**
```
coverage_report/html/index.html
```

在浏览器中打开 HTML 报告可以查看整体覆盖率百分比、每个文件的覆盖率和未覆盖的代码行。

**测试文件位置：**
- 轨迹控制器单元测试：`src/arm_controller/test/test_topic_subscription_lifecycle.cpp`
- 速度控制器单元测试：`src/arm_controller/test/test_velocity_controller_subscriptions.cpp`
- 控制器节点单元测试：`src/arm_controller/test/test_arm_controller_node.cpp`

**提交 Pull Request 前必须：**
- ✅ 所有单元测试通过（运行 `./run_tests.sh`）
- ✅ 为新功能编写对应的单元测试
- ✅ 单元测试覆盖率 ≥ 95%（可用 `./coverage_report.sh` 验证）

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
>>>>>>> develop
   ```bash
   git clone https://github.com/Ding-Kaiyue/universal-arm-controller.git
   cd universal-arm-controller
   ```

2. **创建功能分支**
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **开发您的功能**
   - 遵循代码规范
   - 添加必要的测试
   - 更新相关文档

4. **提交更改**
   ```bash
   git add .
   git commit -m "feat: add your feature description"
   ```

5. **创建 Pull Request**
   - 详细描述您的更改
   - 包含测试结果
   - 等待代码审查

### 代码规范

- 使用 C++17 标准
- 遵循 Google C++ 风格指南
- 添加必要的注释
- 确保代码通过所有测试

### 提交信息格式

```
type(scope): description

[optional body]

[optional footer]
```

类型包括：
- `feat`: 新功能
- `fix`: Bug 修复
- `docs`: 文档更新
- `style`: 代码格式
- `refactor`: 重构
- `test`: 测试相关
- `chore`: 构建过程或辅助工具的变动

### 报告问题和建议

我们提供了标准化的 Issue 模板来帮助您更有效地沟通：

- **[🐛 报告 Bug](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=bug_report.md)**
- **[✨ 提出功能请求](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=feature_request.md)**
- **[❓ 提出使用问题](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=usage_question.md)**
- **[🔒 报告安全漏洞](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=security_report.md)**

### 联系方式

- **Issues**: 通过 GitHub Issues
- **Pull Requests**: 通过 GitHub PR
- **商业合作**: kaiyue.ding@raysense.com
- **技术支持**: d18292819833 (微信)

感谢您的贡献！ 
