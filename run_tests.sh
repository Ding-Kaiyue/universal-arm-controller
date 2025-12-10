#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Unit Test Runner & Pre-commit Validator${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

echo -e "${YELLOW}Building all packages with testing enabled...${NC}"
echo ""

# 步骤1：编译所有包
echo -e "${BLUE}[Step 1/3] Building all packages with testing enabled...${NC}"
colcon build --cmake-args -DBUILD_TESTING=ON 2>&1

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Build failed!${NC}"
    echo -e "${YELLOW}Please fix compilation errors before running tests.${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Build successful!${NC}"
echo ""

# 步骤2：运行单元测试
echo -e "${BLUE}[Step 2/3] Running unit tests (gtest only)...${NC}"
echo ""

# 进入build/arm_controller目录并运行测试
ORIGINAL_DIR=$(pwd)
cd build/arm_controller

# 运行单元测试（只有 gtest 会显示详细输出）
# 使用 -L gtest 标签过滤来只运行 gtest，排除代码风格检查等
ctest -L gtest --output-on-failure 2>&1

CTEST_RESULT=$?

# 返回原始目录
cd ${ORIGINAL_DIR}

echo ""

if [ $CTEST_RESULT -ne 0 ]; then
    echo -e "${RED}❌ Unit tests failed!${NC}"
    echo -e "${YELLOW}Please review the test failures above and fix them.${NC}"
    exit 1
fi

echo -e "${GREEN}✅ All unit tests passed!${NC}"
echo ""

# 步骤3：显示测试统计
echo -e "${BLUE}[Step 3/3] Test summary...${NC}"
echo ""
echo -e "${GREEN}✅ Compilation: PASSED${NC}"
echo -e "${GREEN}✅ Unit Tests: PASSED${NC}"
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}All checks passed! Ready to commit.${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 显示提交前的检查清单
echo -e "${BLUE}Pre-commit checklist:${NC}"
echo -e "${GREEN}  ✅ Code compiles successfully${NC}"
echo -e "${GREEN}  ✅ All unit tests pass${NC}"
echo -e "${YELLOW}  ⚠️  Code style check (run separately if needed)${NC}"
echo ""

echo -e "${YELLOW}Next steps:${NC}"
echo "  1. Review your changes: git status"
echo "  2. Stage changes: git add ."
echo "  3. Create commit: git commit -m 'your message'"
echo "  4. Push to remote: git push origin <branch-name>"
echo ""

exit 0
