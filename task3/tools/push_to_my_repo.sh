#!/usr/bin/env bash
# 把本任务（task3/）同步到某个远端仓库的对应目录并推送。
#
#   bash push_to_my_repo.sh                 # 默认推到个人仓库的 Task3/
#   bash push_to_my_repo.sh "fix: ..."      # 自定义提交信息
#
# 切换目标仓库/目录（用环境变量覆盖）：
#   # 推到小组仓库（目录名是小写 task3）
#   TASK3_PUSH_REMOTE=git@github.com:chixin114514/2026Summer_IntegratedRobotProgram_Group-Team-fullcans.git \
#   TASK3_PUSH_SUBDIR=task3 \
#   TASK3_PUSH_MIRROR="$HOME/.cache/task3_group_mirror" \
#   bash push_to_my_repo.sh "feat(task3): ..."
#
# 为什么不能直接 `git push`：
#   本地这个仓库是小组仓库的克隆（根目录 README.md / tak2 / task3）；
#   个人仓库的根目录是 Task1 / Task2 / Task3。两个仓库的历史和布局都不同，
#   git 远程只能一对一映射整个仓库，没法把 task3/ 映射到 Task3/。
#   所以直接推 origin main 会被拒（历史无关），一旦加 --force 就会把
#   Task1、Task2 整个删掉。这个脚本改成：维护一份目标仓库的临时检出，
#   把 task3/ 的内容同步进它的对应目录，再提交推送。
#
# 注意：不用 --delete。目标仓库里如果还有本地没有的文件（比如队友新加的），
# 会被保留下来，不会被这次同步删掉。
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK3="$(dirname "$HERE")"
REMOTE="${TASK3_PUSH_REMOTE:-git@github.com:boff868/2026Summer-integrated-robot-grouptask.git}"
SUBDIR="${TASK3_PUSH_SUBDIR:-Task3}"
BRANCH="${TASK3_PUSH_BRANCH:-main}"
MIRROR="${TASK3_PUSH_MIRROR:-$HOME/.cache/task3_push_mirror}"

echo "目标仓库: $REMOTE"
echo "目标目录: $SUBDIR/   （本地 $TASK3/）"
echo "临时检出: $MIRROR"
echo

if [ -d "$MIRROR/.git" ]; then
  git -C "$MIRROR" fetch -q origin "$BRANCH"
  git -C "$MIRROR" checkout -q "$BRANCH"
  git -C "$MIRROR" reset -q --hard "origin/$BRANCH"
else
  mkdir -p "$(dirname "$MIRROR")"
  git clone -q "$REMOTE" "$MIRROR"
fi

mkdir -p "$MIRROR/$SUBDIR"

rsync -a \
  --exclude '.git' --exclude '.DS_Store' --exclude '__pycache__' \
  --exclude '*.pyc' --exclude '*.pyo' \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  "$TASK3/" "$MIRROR/$SUBDIR/"

git -C "$MIRROR" add -A
if git -C "$MIRROR" diff --cached --quiet; then
  echo "已是最新，没有需要推送的改动。"
  exit 0
fi

echo "本次改动:"
git -C "$MIRROR" diff --cached --stat | tail -20
echo

git -C "$MIRROR" commit -q -m "${1:-sync task3 source}"
git -C "$MIRROR" push origin "$BRANCH"
echo "已推送 -> $REMOTE ($BRANCH)"
