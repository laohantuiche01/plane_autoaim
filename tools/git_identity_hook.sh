#!/bin/bash
# Git 自动身份配置 Hook
# 在进入 ckyf_vision 目录时自动配置 git 用户名
# 添加到 ~/.bashrc 或 ~/.zshrc

setup_ckyf_git_identity() {
    # 检查是否进入了 ckyf_vision 项目
    if [[ "$PWD" == */ckyf_vision* ]]; then
        # 只在首次进入或切换目录时运行一次
        if [ "$_CKYF_GIT_SETUP_DIR" != "$PWD" ]; then
            _CKYF_GIT_SETUP_DIR="$PWD"

            # 查找项目根目录
            local project_root="$PWD"
            while [ "$project_root" != "/" ]; do
                if [ -f "$project_root/.envrc" ] && [ -d "$project_root/src/robot_bringup" ]; then
                    break
                fi
                project_root=$(dirname "$project_root")
            done

            if [ "$project_root" != "/" ]; then
                # 自动运行 Git 身份配置
                bash "$project_root/tools/setup_git_identity.sh" "$project_root" >/dev/null 2>&1
            fi
        fi
    fi
}

# 每次执行命令前检查（可选，会有性能影响）
# 若要启用，取消注释下一行
# PROMPT_COMMAND="setup_ckyf_git_identity;${PROMPT_COMMAND}"

# 或者手动添加到 cd 的 hook（更高效）
if [ -n "$BASH_VERSION" ]; then
    # Bash
    cd() {
        builtin cd "$@" && setup_ckyf_git_identity
    }
elif [ -n "$ZSH_VERSION" ]; then
    # Zsh
    chpwd_functions+=( setup_ckyf_git_identity )
fi
