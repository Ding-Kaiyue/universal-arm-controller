# 分支保护配置指南

## 📋 概述

本仓库使用分支保护规则来确保代码质量和安全性。受保护的分支包括：
- `master`: 生产就绪的代码
- `develop`: 开发集成的代码

## 🔒 保护规则

### master 分支
- ✅ 需要 Pull Request
- ✅ 需要至少 **2 个核准** (LGTM)
- ✅ 需要 Code Owners 核准
- ✅ 新提交时驳回旧的核准
- ✅ CI/CD 必须通过
- ❌ 禁止 force push
- ❌ 禁止删除分支

### develop 分支
- ✅ 需要 Pull Request
- ✅ 需要至少 **2 个核准** (LGTM)
- ✅ 需要 Code Owners 核准
- ✅ 新提交时驳回旧的核准
- ✅ CI/CD 必须通过
- ❌ 禁止 force push
- ❌ 禁止删除分支

## 🚀 自动应用分支保护

### 前置条件

1. **安装 GitHub CLI**
   ```bash
   # macOS
   brew install gh

   # Ubuntu/Debian
   sudo apt-get install gh

   # Windows
   choco install gh

   # 或访问官网下载: https://cli.github.com/
   ```

2. **认证到 GitHub**
   ```bash
   gh auth login
   # 选择 GitHub.com
   # 选择 HTTPS
   # 使用浏览器登录或输入 Personal Access Token
   ```

3. **拥有仓库管理员权限**

### 执行脚本

在仓库根目录运行：

```bash
bash .github/protect-branches.sh
```

脚本将：
1. 检查 GitHub CLI 是否安装和认证
2. 为 `master` 分支应用保护规则
3. 为 `develop` 分支应用保护规则
4. 显示当前保护状态

### 输出示例

```
📦 目标仓库: Ding-Kaiyue/universal-arm-controller

🚀 开始应用分支保护规则...

🔒 保护分支: master
  → 要求 PR (需要 2 个核准)...
  ✅ 分支 master 保护成功

🔒 保护分支: develop
  → 要求 PR (需要 2 个核准)...
  ✅ 分支 develop 保护成功

📊 保护结果总结:
  ✅ master 分支
  ✅ develop 分支

✅ 所有分支保护规则已成功应用!
```

## 📝 配置文件

### settings.yml
主配置文件，记录了分支保护的完整规则。这个文件可以用于：
- 文档化分支保护规则
- 版本控制配置历史
- 作为 Probot repo-settings 应用的配置

### protect-branches.sh
自动应用分支保护的 bash 脚本。使用 GitHub API 通过 GitHub CLI 直接配置。

## 🔐 验证保护

### 在 GitHub 网站上验证

1. 进入仓库主页
2. 点击 Settings
3. 点击 Branches
4. 查看 "Branch protection rules" 部分

你应该看到：
- master 分支受保护
- develop 分支受保护

### 通过 GitHub CLI 验证

```bash
# 查看 master 保护规则
gh api repos/{owner}/{repo}/branches/master/protection

# 查看 develop 保护规则
gh api repos/{owner}/{repo}/branches/develop/protection
```

## ❓ 故障排除

### 问题：GitHub CLI 未安装
**解决方案**：访问 https://cli.github.com/ 安装 GitHub CLI

### 问题：认证失败
**解决方案**：运行 `gh auth login` 重新认证

### 问题：权限不足
**解决方案**：确保你拥有仓库管理员权限

### 问题：API 调用返回 404
**解决方案**：确保分支名称正确（master/develop）

## 📖 更多信息

- [GitHub Branch Protection 文档](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-protected-branches)
- [GitHub CLI 文档](https://cli.github.com/manual/)
- [GitHub Code Owners](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/about-code-owners)

## 🤝 贡献指南

请参考 [CONTRIBUTING.md](./CONTRIBUTING.md) 了解如何提交 PR 和遵守分支保护规则。
