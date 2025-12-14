# 解决SSH主机名解析问题

## 问题分析

当您执行 `git push raspberry main` 命令时，遇到了以下错误：

```
ssh: Could not resolve hostname hanfeipi: Name or service not known
fatal: Could not read from remote repository.
```

这个错误表明您的本地开发机无法解析 `hanfeipi` 这个主机名。这可能是因为：

1. 本地开发机的 `/etc/hosts` 文件中没有 `hanfeipi` 的条目
2. 网络中没有DNS服务器能够解析 `hanfeipi`
3. 树莓派的主机名不是 `hanfeipi`

## 解决方案

### 步骤1：获取树莓派的IP地址

首先，您需要获取树莓派的实际IP地址。

在树莓派上执行以下命令：

```bash
ip addr show
```

查找以 `inet` 开头的行，通常是在 `eth0`（有线连接）或 `wlan0`（无线连接）接口下。

例如：
```
inet 192.168.1.100/24 brd 192.168.1.255 scope global dynamic eth0
```

这里的 `192.168.1.100` 就是树莓派的IP地址。

### 步骤2：测试SSH连接（使用IP地址）

在本地开发机上，使用获取到的IP地址测试SSH连接：

```bash
ssh hanfei@192.168.1.100
```

请将 `192.168.1.100` 替换为您树莓派的实际IP地址。

### 步骤3：修改Git远程仓库配置

如果SSH连接成功，您需要修改Git远程仓库配置，使用IP地址代替主机名：

```bash
# 首先删除现有的远程仓库配置
cd e:\cv-ros2-px4\cv-ros2-px4/
git remote remove raspberry

# 添加新的远程仓库配置（使用IP地址）
git remote add raspberry hanfei@192.168.1.100:~/cv-ros2-px4/

# 验证远程仓库配置
# 对于PowerShell 5，请使用分号代替&&
cd e:\cv-ros2-px4\cv-ros2-px4/; git remote -v
```

### 步骤4：重新尝试推送更改

现在，您可以重新尝试将本地更改推送到树莓派：

```bash
cd e:\cv-ros2-px4\cv-ros2-px4/
git push raspberry main
```

## 高级配置（可选）

### 选项1：在本地开发机的hosts文件中添加主机名条目

如果您想继续使用 `hanfeipi` 这个主机名，可以在本地开发机的hosts文件中添加条目：

对于Windows系统：
1. 以管理员身份打开命令提示符
2. 编辑hosts文件：
   ```
   notepad C:\Windows\System32\drivers\etc\hosts
   ```
3. 添加以下条目：
   ```
   192.168.1.100 hanfeipi
   ```
4. 保存并关闭文件

对于Linux/macOS系统：
```bash
sudo nano /etc/hosts
```
添加相同的条目，然后保存并关闭文件。

### 选项2：配置SSH别名

您还可以在 `~/.ssh/config` 文件中配置SSH别名：

```bash
nano ~/.ssh/config
```

添加以下内容：

```
Host hanfeipi
    HostName 192.168.1.100
    User hanfei
    Port 22
    IdentityFile ~/.ssh/id_rsa
```

保存并关闭文件。现在您可以使用 `hanfeipi` 这个别名来连接树莓派。

## 故障排除

如果您仍然遇到问题，请检查：

1. 树莓派的IP地址是否正确
2. 树莓派和本地开发机是否在同一网络中
3. 树莓派的SSH服务是否正在运行：
   ```bash
   sudo systemctl status ssh
   ```
4. 树莓派的防火墙是否允许SSH连接：
   ```bash
   sudo ufw status
   ```

如果您需要更多帮助，请随时提问！