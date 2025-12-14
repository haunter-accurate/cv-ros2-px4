# 通过SSH直接同步本地仓库到树莓派

## 问题背景

当GitHub暂时不可用时，您可以通过SSH直接将本地开发机上的Git仓库更改推送到树莓派上的仓库，无需经过GitHub中转。

## 解决方案概述

1. **确保树莓派上的仓库配置正确**
2. **在本地开发机上添加树莓派作为远程仓库**
3. **将本地更改推送到树莓派上的仓库**
4. **在树莓派上验证更改**

## 详细步骤

### 步骤1：检查树莓派上的仓库配置

首先，在树莓派上检查您的Git仓库状态：

```bash
# 在树莓派上执行
cd ~/cv-ros2-px4/
git status
```

### 步骤2：在树莓派上设置Git仓库（如果需要）

如果树莓派上的仓库已经存在并且是一个有效的Git仓库，可以跳过此步骤。否则，在树莓派上初始化一个Git仓库：

```bash
# 在树莓派上执行
mkdir -p ~/cv-ros2-px4/
cd ~/cv-ros2-px4/
git init
```

### 步骤3：在本地开发机上添加树莓派作为远程仓库

在本地开发机上，使用SSH添加树莓派作为Git远程仓库：

```bash
# 在本地开发机上执行
cd e:\cv-ros2-px4\cv-ros2-px4/

# 添加树莓派作为远程仓库（命名为"raspberry"）
git remote add raspberry hanfei@hanfeipi:~/cv-ros2-px4/

# 验证远程仓库是否添加成功
git remote -v
```

### 步骤4：将本地更改推送到树莓派

确保您在本地开发机上已经提交了所有更改，然后将更改推送到树莓派上的仓库：

```bash
# 在本地开发机上执行

# 确保所有更改都已提交
git status
git add .
git commit -m "Sync changes to raspberry pi"

# 推送到树莓派上的仓库（推送到master分支）
git push raspberry master

# 如果您使用的是main分支，使用以下命令
git push raspberry main
```

### 步骤5：在树莓派上验证更改

在树莓派上检查更改是否已经成功同步：

```bash
# 在树莓派上执行
cd ~/cv-ros2-px4/
git status
git log -n 5
```

### 步骤6：定期同步更改

您可以使用以下命令定期将本地更改同步到树莓派上的仓库：

```bash
# 在本地开发机上执行
cd e:\cv-ros2-px4\cv-ros2-px4/
git add .
git commit -m "Sync changes to raspberry pi"
git push raspberry master
```

## 高级配置（可选）

### 设置SSH别名

为了方便使用，您可以在本地开发机的`~/.ssh/config`文件中设置SSH别名：

```bash
# 在本地开发机上执行
nano ~/.ssh/config
```

添加以下内容：

```
Host hanfeipi
    HostName hanfeipi
    User hanfei
    Port 22
    IdentityFile ~/.ssh/id_rsa
```

保存并退出。现在您可以使用更简单的命令：

```bash
git remote add raspberry hanfeipi:~/cv-ros2-px4/
```

### 设置树莓派上的仓库为Bare仓库

如果您只需要将树莓派上的仓库作为同步目标，而不在那里直接进行开发，可以将其设置为bare仓库：

```bash
# 在树莓派上执行
cd ~/
rm -rf ~/cv-ros2-px4/
git init --bare ~/cv-ros2-px4.git
```

然后在本地开发机上添加远程仓库：

```bash
# 在本地开发机上执行
git remote add raspberry hanfei@hanfeipi:~/cv-ros2-px4.git
```

### 从树莓派拉取更改到本地开发机

如果您在树莓派上进行了更改，也可以将这些更改拉取到本地开发机：

```bash
# 在本地开发机上执行
cd e:\cv-ros2-px4\cv-ros2-px4/
git pull raspberry master
```

## 注意事项

1. 确保您的本地开发机和树莓派之间可以通过SSH正常通信
2. 确保树莓派上的Git仓库路径正确
3. 确保您在树莓派上有足够的权限写入仓库
4. 如果遇到SSH连接问题，请检查网络连接和SSH配置
5. 当GitHub恢复可用后，您可以使用正常的Git流程与GitHub同步

## 故障排除

### 问题：SSH连接失败

如果您无法通过SSH连接到树莓派，请检查：

```bash
# 在本地开发机上执行
# 测试SSH连接
ssh hanfei@hanfeipi

# 如果使用了SSH密钥，请检查密钥是否正确配置
ssh -i ~/.ssh/id_rsa hanfei@hanfeipi
```

### 问题：Git推送失败

如果Git推送失败，请检查：

```bash
# 在树莓派上执行
# 确保树莓派上的仓库不是bare仓库，或者配置了接收推送
cd ~/cv-ros2-px4/
git config receive.denyCurrentBranch updateInstead
```

### 问题：文件权限问题

如果遇到文件权限问题，请检查：

```bash
# 在树莓派上执行
# 修复仓库的权限
cd ~/cv-ros2-px4/
chmod -R 755 .git/
```

## 总结

通过以上步骤，您可以在GitHub不可用的情况下，通过SSH直接将本地开发机上的Git仓库更改推送到树莓派上的仓库，实现两者之间的同步。当GitHub恢复可用后，您可以继续使用正常的Git流程与GitHub同步。