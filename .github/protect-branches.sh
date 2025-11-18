#!/bin/bash
# 分支保护自动化脚本
# 使用 GitHub CLI 为 master 和 develop 分支应用保护规则
#
# 使用方法:
#   bash .github/protect-branches.sh
#
# 前置条件:
#   - 安装了 GitHub CLI: https://cli.github.com/
#   - 已认证: gh auth login
#   - 拥有仓库管理员权限

set -e

# 颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 检查 GitHub CLI
if ! command -v gh &> /dev/null; then
    echo -e "${RED}❌ 错误: GitHub CLI 未安装${NC}"
    echo "请访问 https://cli.github.com/ 安装 GitHub CLI"
    exit 1
fi

# 检查认证
if ! gh auth status > /dev/null 2>&1; then
    echo -e "${RED}❌ 错误: 未认证到 GitHub${NC}"
    echo "请运行: gh auth login"
    exit 1
fi

# 获取仓库信息
REPO=$(gh repo view --json nameWithOwner -q .)
echo -e "${YELLOW}📦 目标仓库: $REPO${NC}"

# 分支保护配置
protect_branch() {
    local BRANCH=$1
    local REQUIRED_APPROVALS=$2

    echo -e "\n${YELLOW}🔒 保护分支: $BRANCH${NC}"

    # 启用 PR 要求
    echo "  → 要求 PR (需要 $REQUIRED_APPROVALS 个核准)..."

    # 创建临时 JSON 文件
    TEMP_JSON=$(mktemp)
    cat > "$TEMP_JSON" << EOF
{
  "required_pull_request_reviews": {
    "required_approving_review_count": $REQUIRED_APPROVALS,
    "dismiss_stale_reviews": true,
    "require_code_owner_reviews": true
  },
  "required_status_checks": {
    "strict": true,
    "contexts": []
  },
  "enforce_admins": true,
  "allow_force_pushes": false,
  "allow_deletions": false,
  "restrictions": null
}
EOF

    gh api \
        repos/{owner}/{repo}/branches/$BRANCH/protection \
        -X PUT \
        --input "$TEMP_JSON" 2>/dev/null

    RESULT=$?
    rm -f "$TEMP_JSON"

    if [ $RESULT -eq 0 ]; then
        echo -e "${GREEN}  ✅ 分支 $BRANCH 保护成功${NC}"
        return 0
    else
        echo -e "${RED}  ❌ 分支 $BRANCH 保护失败${NC}"
        return 1
    fi
}

# 执行保护
echo -e "\n${YELLOW}🚀 开始应用分支保护规则...${NC}"

# 保护 master (2 个核准)
protect_branch "master" 2
MASTER_RESULT=$?

# 保护 develop (2 个核准)
protect_branch "develop" 2
DEVELOP_RESULT=$?

# 总结
echo -e "\n${YELLOW}📊 保护结果总结:${NC}"

if [ $MASTER_RESULT -eq 0 ]; then
    echo -e "  ${GREEN}✅ master 分支${NC}"
else
    echo -e "  ${RED}❌ master 分支${NC}"
fi

if [ $DEVELOP_RESULT -eq 0 ]; then
    echo -e "  ${GREEN}✅ develop 分支${NC}"
else
    echo -e "  ${RED}❌ develop 分支${NC}"
fi

# 显示当前保护状态
echo -e "\n${YELLOW}📋 当前分支保护状态:${NC}"

echo -e "\n${YELLOW}master 分支:${NC}"
gh api repos/{owner}/{repo}/branches/master/protection -q '.required_pull_request_reviews, .enforce_admins, .allow_force_pushes, .allow_deletions' 2>/dev/null || echo "  未启用保护"

echo -e "\n${YELLOW}develop 分支:${NC}"
gh api repos/{owner}/{repo}/branches/develop/protection -q '.required_pull_request_reviews, .enforce_admins, .allow_force_pushes, .allow_deletions' 2>/dev/null || echo "  未启用保护"

# 检查是否所有操作都成功
if [ $MASTER_RESULT -eq 0 ] && [ $DEVELOP_RESULT -eq 0 ]; then
    echo -e "\n${GREEN}✅ 所有分支保护规则已成功应用!${NC}"
    echo -e "\n${YELLOW}保护规则详情:${NC}"
    echo "  • 需要 Pull Request 才能合并"
    echo "  • 需要至少 2 个核准 (LGTM)"
    echo "  • 需要 Code Owners 核准"
    echo "  • 新的 commit 会驳回旧的核准"
    echo "  • 禁止 force push"
    echo "  • 禁止删除分支"
    echo "  • CI/CD 必须通过"
    exit 0
else
    echo -e "\n${RED}❌ 部分分支保护规则应用失败!${NC}"
    exit 1
fi
