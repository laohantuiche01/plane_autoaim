#!/bin/bash
# ckyf_vision 项目环境配置脚本
# 用法: bash tools/setup_env.sh

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

print_status() {
    if dpkg -s "$1" &>/dev/null; then
        echo -e "  ${GREEN}[已安装]${NC} $1"
        return 0
    else
        echo -e "  ${RED}[未安装]${NC} $1"
        return 1
    fi
}

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  ckyf_vision 环境配置脚本${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# ============================================================
# 1. ROS 2 检查
# ============================================================
echo -e "${YELLOW}[1/5] 检查 ROS 2 环境${NC}"
if [ -n "$ROS_DISTRO" ]; then
    echo -e "  ${GREEN}[已安装]${NC} ROS 2 $ROS_DISTRO"
else
    echo -e "  ${RED}[未检测到]${NC} ROS 2 (请先安装 ROS 2 Humble 并 source setup.bash)"
    echo -e "  参考: https://docs.ros.org/en/humble/Installation.html"
fi
echo ""

# ============================================================
# 2. 系统依赖库
# ============================================================
echo -e "${YELLOW}[2/5] 检查系统依赖库${NC}"

DEPS=(
    # OpenCV
    "libopencv-dev"
    # Eigen3
    "libeigen3-dev"
    # G2O
    "libg2o-dev"
    # fmt
    "libfmt-dev"
    # TBB
    "libtbb-dev"
    # Sophus (可能需要从源码安装)
    "libsophus-dev"
)

MISSING=()
for dep in "${DEPS[@]}"; do
    if ! print_status "$dep"; then
        MISSING+=("$dep")
    fi
done

echo ""
if [ ${#MISSING[@]} -gt 0 ]; then
    echo -e "${YELLOW}发现 ${#MISSING[@]} 个未安装的依赖:${NC}"
    echo "  ${MISSING[*]}"
    echo ""
    read -p "是否自动安装这些依赖? [y/N] " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt update
        sudo apt install -y "${MISSING[@]}"
        echo -e "${GREEN}依赖安装完成${NC}"
    fi
else
    echo -e "${GREEN}所有系统依赖已就绪${NC}"
fi
echo ""

# ============================================================
# 3. ROS 2 相关包
# ============================================================
echo -e "${YELLOW}[3/5] 检查 ROS 2 依赖包${NC}"

ROS_DEPS=(
    "ros-${ROS_DISTRO:-humble}-tf2-geometry-msgs"
    "ros-${ROS_DISTRO:-humble}-cv-bridge"
    "ros-${ROS_DISTRO:-humble}-image-transport"
    "ros-${ROS_DISTRO:-humble}-rclcpp-components"
    "ros-${ROS_DISTRO:-humble}-lifecycle-msgs"
    "ros-${ROS_DISTRO:-humble}-rclcpp-lifecycle"
)

ROS_MISSING=()
for dep in "${ROS_DEPS[@]}"; do
    if ! print_status "$dep"; then
        ROS_MISSING+=("$dep")
    fi
done

echo ""
if [ ${#ROS_MISSING[@]} -gt 0 ]; then
    echo -e "${YELLOW}发现 ${#ROS_MISSING[@]} 个未安装的 ROS 2 包:${NC}"
    echo "  ${ROS_MISSING[*]}"
    echo ""
    read -p "是否自动安装? [y/N] " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt install -y "${ROS_MISSING[@]}"
        echo -e "${GREEN}ROS 2 依赖安装完成${NC}"
    fi
else
    echo -e "${GREEN}所有 ROS 2 依赖已就绪${NC}"
fi
echo ""

# ============================================================
# 4. Iceoryx (零拷贝 IPC 中间件)
# ============================================================
echo -e "${YELLOW}[4/5] 检查 Iceoryx 中间件${NC}"

if command -v iox-roudi &>/dev/null; then
    echo -e "  ${GREEN}[已安装]${NC} Iceoryx RouDi"
else
    echo -e "  ${RED}[未安装]${NC} Iceoryx RouDi"
    echo ""
    echo "  Iceoryx 用于进程内零拷贝通信，推荐从源码安装:"
    echo "    git clone https://github.com/eclipse-iceoryx/iceoryx.git"
    echo "    cd iceoryx && cmake -Bbuild -Hiceoryx_meta -DCMAKE_INSTALL_PREFIX=/usr/local"
    echo "    cmake --build build -j\$(nproc) && sudo cmake --install build"
    echo ""
    read -p "是否尝试自动从源码编译安装 Iceoryx? [y/N] " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        TMPDIR=$(mktemp -d)
        git clone --depth 1 https://github.com/eclipse-iceoryx/iceoryx.git "$TMPDIR/iceoryx"
        cd "$TMPDIR/iceoryx"
        cmake -Bbuild -Hiceoryx_meta -DCMAKE_INSTALL_PREFIX=/usr/local
        cmake --build build -j"$(nproc)"
        sudo cmake --install build
        cd -
        rm -rf "$TMPDIR"
        echo -e "${GREEN}Iceoryx 安装完成${NC}"
    fi
fi
echo ""

# ============================================================
# 5. 海康机器人 MVS SDK (相机驱动)
# ============================================================
echo -e "${YELLOW}[5/5] 检查海康 MVS SDK${NC}"

if [ -d "/opt/MVS" ]; then
    echo -e "  ${GREEN}[已安装]${NC} MVS SDK (/opt/MVS)"
else
    echo -e "  ${RED}[未安装]${NC} MVS SDK"
    echo ""
    echo -e "  ${YELLOW}海康 MVS SDK 需要手动下载:${NC}"
    echo "  1. 访问: https://www.hikrobotics.com/cn/machinevision/service/download/?module=0"
    echo "  2. 下载 Linux 版本的 MVS 客户端安装包 (.zip)"
    echo "  3. 下载完成后将 .zip 文件放到当前目录"
    echo ""
    read -p "下载完成后按回车继续安装，或输入 s 跳过: " -n 1 -r
    echo ""

    if [[ ! $REPLY =~ ^[Ss]$ ]]; then
        # 查找 zip 文件
        MVS_ZIP=$(ls MVS_STD_GML_V*.zip 2>/dev/null | head -1)
        if [ -z "$MVS_ZIP" ]; then
            read -p "请输入 MVS zip 文件路径: " MVS_ZIP
        fi

        if [ -f "$MVS_ZIP" ]; then
            echo "正在解压 $MVS_ZIP ..."
            unzip -o "$MVS_ZIP" -d /tmp/mvs_install

            ARCH=$(uname -m)
            echo "检测到系统架构: $ARCH"

            if [ "$ARCH" == "x86_64" ]; then
                DEB=$(find /tmp/mvs_install -name "MVS-*_x86_64_*.deb" | head -1)
            elif [ "$ARCH" == "aarch64" ]; then
                DEB=$(find /tmp/mvs_install -name "MVS-*_aarch64_*.deb" | head -1)
            elif [ "$ARCH" == "i386" ]; then
                DEB=$(find /tmp/mvs_install -name "MVS-*_i386_*.deb" | head -1)
            fi

            if [ -n "$DEB" ] && [ -f "$DEB" ]; then
                echo "安装: $DEB"
                sudo dpkg -i "$DEB"
                sudo apt-get install -f -y  # 修复可能的依赖问题
                echo -e "${GREEN}MVS SDK 安装完成${NC}"
            else
                echo -e "${RED}未找到匹配架构 ($ARCH) 的 .deb 安装包${NC}"
                echo "请手动安装"
            fi
            rm -rf /tmp/mvs_install
        else
            echo -e "${RED}文件不存在: $MVS_ZIP${NC}"
        fi
    fi
fi
echo ""

# ============================================================
# 汇总
# ============================================================
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  环境检查汇总${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

ALL_OK=true

check_final() {
    if $2; then
        echo -e "  ${GREEN}✓${NC} $1"
    else
        echo -e "  ${RED}✗${NC} $1"
        ALL_OK=false
    fi
}

check_final "ROS 2"          "[ -n \"$ROS_DISTRO\" ]"
check_final "OpenCV"         "dpkg -s libopencv-dev &>/dev/null"
check_final "Eigen3"         "dpkg -s libeigen3-dev &>/dev/null"
check_final "G2O"            "dpkg -s libg2o-dev &>/dev/null"
check_final "fmt"            "dpkg -s libfmt-dev &>/dev/null"
check_final "TBB"            "dpkg -s libtbb-dev &>/dev/null"
check_final "Iceoryx"        "command -v iox-roudi &>/dev/null"
check_final "MVS SDK"        "[ -d /opt/MVS ]"

echo ""
if $ALL_OK; then
    echo -e "${GREEN}所有依赖已就绪! 可以开始构建:${NC}"
    echo "  cd $(dirname "$0")/.."
    echo "  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
else
    echo -e "${YELLOW}部分依赖缺失，请根据上方提示安装后重新运行此脚本${NC}"
fi