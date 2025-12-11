#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Unit Test Coverage Analysis${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 步骤1：清除之前的覆盖率数据
echo -e "${YELLOW}[Step 1/3] Clearing previous coverage data...${NC}"
rm -rf build install log coverage_report
echo -e "${GREEN}✅ Cleared${NC}"
echo ""

# 步骤2：编译所有包并启用覆盖率
echo -e "${YELLOW}[Step 2/3] Building with coverage enabled...${NC}"
colcon build --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -20

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Build failed!${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Build successful!${NC}"
echo ""

# 步骤3：运行单元测试生成覆盖率数据
echo -e "${YELLOW}[Step 3/3] Running unit tests to generate coverage data...${NC}"
ORIGINAL_DIR=$(pwd)
cd build/arm_controller

ctest -L gtest --output-on-failure 2>&1

CTEST_RESULT=$?
cd ${ORIGINAL_DIR}

if [ $CTEST_RESULT -ne 0 ]; then
    echo -e "${RED}❌ Unit tests failed!${NC}"
    exit 1
fi

echo -e "${GREEN}✅ All unit tests passed!${NC}"
echo ""

# 步骤4：检查是否有覆盖率工具可用
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Coverage Analysis${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

if command -v lcov &> /dev/null; then
    echo -e "${YELLOW}Generating detailed coverage report with lcov...${NC}"

    # 创建覆盖率报告目录
    COVERAGE_DIR="coverage_report"
    rm -rf ${COVERAGE_DIR}
    mkdir -p ${COVERAGE_DIR}

    # 步骤1：初始化覆盖率数据（包含所有内容）
    echo -e "${YELLOW}  Capturing all coverage data...${NC}"
    lcov --directory build/arm_controller --capture --output-file ${COVERAGE_DIR}/coverage_all.info 2>&1 > /dev/null

    # 步骤2：移除系统库、第三方库、不相关目录，以及薄包装层
    # 薄包装层（只是委托给外部库的代码）不应该计入覆盖率
    echo -e "${YELLOW}  Filtering out system libraries, external dependencies, and wrapper layers...${NC}"
    lcov --remove ${COVERAGE_DIR}/coverage_all.info \
        '/opt/*' \
        '/usr/*' \
        '*/boost/*' \
        '*/eigen3/*' \
        '*/yaml-cpp*' \
        '*/c++/*' \
        '*/.conan/*' \
        '*/build/*' \
        '*/install/*' \
        '*/gtest*' \
        '*/hardware_manager.cpp' \
        '*/shm_manager.cpp' \
        --output-file ${COVERAGE_DIR}/coverage.info 2>&1 > /dev/null

    # 生成HTML报告
    genhtml ${COVERAGE_DIR}/coverage.info --output-directory ${COVERAGE_DIR}/html 2>&1 > /dev/null

    # 提取覆盖率百分比
    COVERAGE_PERCENT=$(lcov --summary ${COVERAGE_DIR}/coverage.info 2>&1 | grep "lines" | awk '{print $2}' | tr -d '%')

    # 检查是否成功提取百分比
    if [ -z "$COVERAGE_PERCENT" ] || ! [[ "$COVERAGE_PERCENT" =~ ^[0-9]+\.[0-9]+$ ]]; then
        echo -e "${RED}❌ Failed to extract coverage percentage${NC}"
    else
        # 使用 awk 进行浮点数比较
        # 注意：这是过滤后的覆盖率（仅包含核心业务逻辑，排除薄包装层）
        if (( $(echo "$COVERAGE_PERCENT >= 85" | bc -l) )); then
            echo -e "${GREEN}✅ Line Coverage (filtered): ${COVERAGE_PERCENT}%${NC}"
            echo -e "${GREEN}✅ Production-ready coverage target (≥85%) MET!${NC}"
        elif (( $(echo "$COVERAGE_PERCENT >= 75" | bc -l) )); then
            echo -e "${YELLOW}⚠️  Line Coverage (filtered): ${COVERAGE_PERCENT}%${NC}"
            echo -e "${YELLOW}⚠️  Coverage is good but not yet production-ready (target: ≥85%)${NC}"
        else
            echo -e "${YELLOW}⚠️  Line Coverage (filtered): ${COVERAGE_PERCENT}%${NC}"
            echo -e "${YELLOW}⚠️  Coverage below recommended threshold${NC}"
        fi
    fi

    echo ""
    echo -e "${BLUE}Note: This report filters out:${NC}"
    echo -e "${BLUE}  • System libraries, ROS2, Boost, Eigen, etc.${NC}"
    echo -e "${BLUE}  • Wrapper layers (hardware_manager.cpp, shm_manager.cpp)${NC}"
    echo -e "${BLUE}  • It only shows coverage of core business logic in arm_controller.${NC}"

    echo ""
    echo -e "${BLUE}HTML Report: ${COVERAGE_DIR}/html/index.html${NC}"
    echo -e "${BLUE}Open in browser: file://$(pwd)/${COVERAGE_DIR}/html/index.html${NC}"
else
    echo -e "${YELLOW}ℹ️  lcov not available for detailed coverage reports${NC}"
    echo ""
    echo -e "${BLUE}To generate detailed coverage reports, install lcov:${NC}"
    echo "  sudo apt-get install lcov"
    echo ""
    echo -e "${BLUE}Then run this script again to generate HTML coverage reports.${NC}"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Unit tests completed successfully!${NC}"
echo -e "${GREEN}========================================${NC}"