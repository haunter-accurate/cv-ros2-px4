# 解决Git推送错误：Working directory has unstaged changes

## 问题分析

当您执行 `git push raspberry main` 命令时，遇到了以下错误：

```
! [remote rejected] main -> main (Working directory has unstaged changes)
error: failed to push some refs to '10.5.89.13:~/cv-ros2-px4/'
```

这个错误表明：

1. 树莓派上的Git仓库工作目录中有**未暂存的更改**（unstaged changes）
2. 虽然您已经配置了 `receive.denyCurrentBranch updateInstead` 允许推送到当前检出的分支，但当工作目录中有未暂存的更改时，Git仍然会拒绝推送
3. 这是因为Git无法安全地自动更新包含未暂存更改的工作目录

## 解决方案

要解决这个问题，您需要在树莓派上处理工作目录中的未暂存更改。以下是几种处理方法：

### 方法1：查看并提交未暂存的更改

如果树莓派上的未暂存更改是您需要保留的，可以将它们提交到本地仓库：

```bash
# 在树莓派上执行
cd ~/cv-ros2-px4/

# 查看工作目录的状态
git status

# 暂存所有更改
git add .

# 提交更改
git commit -m "Save unstaged changes before receiving push"
```

### 方法2：暂存未暂存的更改

如果您不想立即提交这些更改，可以将它们暂存（stash）起来：

```bash
# 在树莓派上执行
cd ~/cv-ros2-px4/

# 暂存所有未暂存的更改
git stash

# 查看所有暂存的更改
git stash list
```

### 方法3：放弃未暂存的更改

如果这些未暂存的更改是您不需要的，可以放弃它们：

```bash
# 在树莓派上执行
cd ~/cv-ros2-px4/

# 放弃工作目录中的所有未暂存更改
git checkout -- .

# 如果有未暂存的新文件，可以使用以下命令删除它们
git clean -fd
```

### 方法4：强制推送（不推荐）

**注意：这个方法不推荐使用，因为它可能会导致数据丢失。**

如果您确定要覆盖树莓派上的所有更改，可以使用强制推送：

```bash
# 在本地开发机上执行
cd e:\cv-ros2-px4\cv-ros2-px4/
git push -f raspberry main
```

强制推送会覆盖树莓派上的所有历史记录和更改，包括未暂存的更改。请谨慎使用这个方法。

## 选择合适的方法

根据您的具体情况，选择合适的方法：

1. 如果树莓派上的未暂存更改是您需要保留的，请选择**方法1**或**方法2**
2. 如果树莓派上的未暂存更改是您不需要的，请选择**方法3**
3. 只有在非常特殊的情况下，才考虑使用**方法4**

## 验证修复

处理完未暂存的更改后，在本地开发机上重新尝试推送：

```bash
# 在本地开发机上执行
cd e:\cv-ros2-px4\cv-ros2-px4/
git push raspberry main
```

推送成功后，在树莓派上验证更改是否已经成功同步：

```bash
# 在树莓派上执行
cd ~/cv-ros2-px4/
git status
git log -n 5
```

## 后续操作

如果您选择了**方法2**（暂存未暂存的更改），并且需要恢复这些更改，可以使用以下命令：

```bash
# 在树莓派上执行
cd ~/cv-ros2-px4/

# 恢复最近一次暂存的更改
git stash pop

# 或者恢复指定的暂存更改
git stash apply stash@{1}
```

## 注意事项

1. 在处理Git仓库的工作目录时，请务必谨慎操作，避免意外丢失重要数据
2. 在执行任何可能导致数据丢失的命令之前，建议先备份您的工作目录
3. 定期提交您的更改，避免工作目录中积累大量未暂存的更改
4. 如果您在团队环境中工作，请确保与其他团队成员协调好Git操作，避免冲突

选择最适合您使用场景的方法，然后按照相应的步骤操作即可解决这个问题。