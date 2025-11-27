#!/bin/bash

# ============================================================================
# shm_clean.sh - 共享内存清理脚本
# 用于清理 arm_controller IPC 的残留资源
# 在以下情况下使用：
# - 程序异常退出残留了 IPC 资源
# - 需要重新初始化 IPC 系统
# - 调试或开发环境重置
# ============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# IPC 资源名称（与 shm_manager.hpp 保持一致）
SHM_NAME="arm_controller_shm_v1"
MUTEX_NAME="arm_controller_mutex"
COND_NAME="arm_controller_cond"

print_header() {
    echo -e "${BLUE}===============================================================================${NC}"
    echo -e "${BLUE}ArmController IPC 清理工具${NC}"
    echo -e "${BLUE}===============================================================================${NC}"
    echo ""
}

print_warning() {
    echo -e "${RED}⚠️  警告${NC}"
    echo "此操作将删除以下所有资源:"
    echo "  - 共享内存: $SHM_NAME"
    echo "  - 互斥量:   $MUTEX_NAME"
    echo "  - 条件变量: $COND_NAME"
    echo ""
    echo -e "${YELLOW}请确保没有 arm_controller 进程在运行！${NC}"
    echo ""
}

confirm() {
    local prompt="$1"
    local response

    while true; do
        echo -n -e "${YELLOW}$prompt (y/N): ${NC}"
        read -r response
        case "$response" in
            [yY][eE][sS]|[yY])
                return 0
                ;;
            [nN][oO]|[nN]|"")
                return 1
                ;;
            *)
                echo "请输入 y 或 n"
                ;;
        esac
    done
}

remove_shm() {
    echo -e "${YELLOW}🔍 搜索共享内存...${NC}"
    local shm_files=$(find /dev/shm -name "*${SHM_NAME}*" 2>/dev/null || true)

    if [ -z "$shm_files" ]; then
        echo -e "${GREEN}✓ 共享内存不存在${NC}"
        return 0
    fi

    echo "找到的共享内存:"
    echo "$shm_files" | sed 's/^/  /'
    echo ""

    echo -e "${YELLOW}删除共享内存...${NC}"
    echo "$shm_files" | while read -r file; do
        rm -f "$file" 2>/dev/null && echo -e "${GREEN}✓ 已删除: $file${NC}" || echo -e "${RED}✗ 失败: $file${NC}"
    done
    echo ""
}

remove_mutex() {
    echo -e "${YELLOW}🔍 搜索互斥量...${NC}"
    local mutex_file="/dev/shm/sem.${MUTEX_NAME}"

    if [ ! -e "$mutex_file" ]; then
        echo -e "${GREEN}✓ 互斥量不存在${NC}"
        return 0
    fi

    echo "找到的互斥量: $mutex_file"
    echo ""

    echo -e "${YELLOW}删除互斥量...${NC}"
    rm -f "$mutex_file" 2>/dev/null && echo -e "${GREEN}✓ 已删除: $mutex_file${NC}" || echo -e "${RED}✗ 失败: $mutex_file${NC}"
    echo ""
}

remove_cond() {
    echo -e "${YELLOW}🔍 搜索条件变量...${NC}"
    local cond_file="/dev/shm/sem.${COND_NAME}"

    if [ ! -e "$cond_file" ]; then
        echo -e "${GREEN}✓ 条件变量不存在${NC}"
        return 0
    fi

    echo "找到的条件变量: $cond_file"
    echo ""

    echo -e "${YELLOW}删除条件变量...${NC}"
    rm -f "$cond_file" 2>/dev/null && echo -e "${GREEN}✓ 已删除: $cond_file${NC}" || echo -e "${RED}✗ 失败: $cond_file${NC}"
    echo ""
}

cleanup() {
    print_header
    print_warning

    if ! confirm "继续清理?"; then
        echo -e "${YELLOW}已取消清理操作${NC}"
        return 0
    fi

    echo ""
    echo -e "${YELLOW}开始清理...${NC}"
    echo ""

    remove_shm
    remove_mutex
    remove_cond

    echo -e "${GREEN}=======================================================================${NC}"
    echo -e "${GREEN}✅ 清理完成！${NC}"
    echo -e "${GREEN}=======================================================================${NC}"
}

dry_run() {
    print_header
    echo -e "${YELLOW}📋 模拟运行模式（不会删除任何文件）${NC}"
    echo ""

    echo "将删除:"
    find /dev/shm -name "*${SHM_NAME}*" 2>/dev/null | sed 's/^/  /' || echo "  (无共享内存)"
    [ -e "/dev/shm/sem.${MUTEX_NAME}" ] && echo "  /dev/shm/sem.${MUTEX_NAME}" || echo "  (无互斥量)"
    [ -e "/dev/shm/sem.${COND_NAME}" ] && echo "  /dev/shm/sem.${COND_NAME}" || echo "  (无条件变量)"

    echo ""
    echo "运行 'shm_clean.sh' 执行真实清理"
}

main() {
    case "$1" in
        --dry-run|-d)
            dry_run
            ;;
        --help|-h)
            echo "用法: shm_clean.sh [选项]"
            echo ""
            echo "选项:"
            echo "  (无)        交互式清理"
            echo "  --dry-run   模拟运行，显示将删除的文件"
            echo "  --help      显示此帮助信息"
            echo ""
            ;;
        *)
            cleanup
            ;;
    esac
}

main "$@"
