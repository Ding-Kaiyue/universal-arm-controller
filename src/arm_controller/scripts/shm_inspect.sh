#!/bin/bash

# ============================================================================
# shm_inspect.sh - 共享内存监控脚本
# 用于检查 arm_controller IPC 共享内存的状态
# ============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'  # No Color

# IPC 资源名称（与 shm_manager.hpp 保持一致）
SHM_NAME="arm_controller_shm_v1"
MUTEX_NAME="arm_controller_mutex"
COND_NAME="arm_controller_cond"

print_header() {
    echo -e "${BLUE}===============================================================================${NC}"
    echo -e "${BLUE}ArmController IPC 共享内存监控工具${NC}"
    echo -e "${BLUE}===============================================================================${NC}"
    echo ""
}

check_shm_exists() {
    # 检查共享内存是否存在（可能有 psm_ 前缀或没有）
    if ls /dev/shm/ 2>/dev/null | grep -qE "(psm_.*)?${SHM_NAME}"; then
        return 0
    fi
    return 1
}

check_mutex_exists() {
    # 检查命名互斥量是否存在
    if [ -e "/dev/shm/sem.${MUTEX_NAME}" ]; then
        return 0
    fi
    return 1
}

check_cond_exists() {
    # 检查命名条件变量是否存在
    if [ -e "/dev/shm/${COND_NAME}" ] || [ -e "/dev/shm/sem.${COND_NAME}" ]; then
        return 0
    fi
    return 1
}

get_shm_size() {
    # 获取共享内存大小
    local shm_file=$(find /dev/shm -name "*${SHM_NAME}*" 2>/dev/null | head -1)
    if [ -n "$shm_file" ]; then
        ls -lh "$shm_file" | awk '{print $5}'
    else
        echo "N/A"
    fi
}

get_shm_owner() {
    # 获取共享内存所有者
    local shm_file=$(find /dev/shm -name "*${SHM_NAME}*" 2>/dev/null | head -1)
    if [ -n "$shm_file" ]; then
        ls -l "$shm_file" | awk '{print $3":"$4}'
    else
        echo "N/A"
    fi
}

get_shm_modified() {
    # 获取共享内存最后修改时间
    local shm_file=$(find /dev/shm -name "*${SHM_NAME}*" 2>/dev/null | head -1)
    if [ -n "$shm_file" ]; then
        stat -c '%y' "$shm_file" 2>/dev/null || stat -f '%Sm' "$shm_file" 2>/dev/null || echo "N/A"
    else
        echo "N/A"
    fi
}

show_status() {
    echo -e "${YELLOW}📊 IPC 资源状态${NC}"
    echo ""

    # 检查共享内存
    if check_shm_exists; then
        echo -e "${GREEN}✅ 共享内存 (SHM)${NC}"
        echo "   名称:   $SHM_NAME"
        echo "   大小:   $(get_shm_size)"
        echo "   所有者: $(get_shm_owner)"
        echo "   修改时间: $(get_shm_modified)"
    else
        echo -e "${RED}❌ 共享内存 (SHM) - 不存在${NC}"
    fi
    echo ""

    # 检查互斥量
    if check_mutex_exists; then
        echo -e "${GREEN}✅ 命名互斥量 (Mutex)${NC}"
        echo "   名称: $MUTEX_NAME"
    else
        echo -e "${RED}❌ 命名互斥量 (Mutex) - 不存在${NC}"
    fi
    echo ""

    # 检查条件变量
    if check_cond_exists; then
        echo -e "${GREEN}✅ 命名条件变量 (Condition)${NC}"
        echo "   名称: $COND_NAME"
    else
        echo -e "${RED}❌ 命名条件变量 (Condition) - 不存在${NC}"
    fi
    echo ""
}

show_ipc_list() {
    echo -e "${YELLOW}📋 所有 IPC 资源列表${NC}"
    echo ""

    echo "共享内存:"
    ls -lh /dev/shm/psm_* 2>/dev/null | grep -E "(arm_controller|psm_)" || echo "  (无)"
    echo ""

    echo "命名互斥量:"
    ls -la /dev/shm/sem.* 2>/dev/null | grep arm_controller || echo "  (无)"
    echo ""

    echo "所有信号量:"
    ipcs -s 2>/dev/null | grep -E "(arm_controller|NSEMS)" || echo "  (无)"
    echo ""
}

show_summary() {
    echo -e "${YELLOW}📈 资源统计${NC}"
    echo ""

    local shm_count=$(ls /dev/shm/*${SHM_NAME}* 2>/dev/null | wc -l)
    local mutex_count=$(ls /dev/shm/sem.arm_controller* 2>/dev/null | wc -l)
    local cond_count=$(ls /dev/shm/arm_controller_cond* 2>/dev/null | wc -l)

    echo "共享内存数: $shm_count"
    echo "命名互斥量数: $mutex_count"
    echo "命名条件变量数: $cond_count"
    echo ""
}

main() {
    print_header

    if [ "$1" == "-v" ] || [ "$1" == "--verbose" ]; then
        show_status
        show_ipc_list
        show_summary
    else
        show_status
        show_summary
    fi

    echo -e "${BLUE}===============================================================================${NC}"
    echo "提示: 运行 'shm_inspect.sh -v' 查看详细信息"
    echo "提示: 运行 'shm_clean.sh' 清理所有 IPC 资源"
    echo -e "${BLUE}===============================================================================${NC}"
}

main "$@"