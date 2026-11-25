
### FTP 访问限制

- 用户名固定为 `root`，密码决定当前会话根目录：`root` 对应 `/userdata/xcharge/log`，`test` 对应 `/mnt/sda`。必须先发送 `USER root` 再发送 `PASS`；错误或缺失凭据返回 `530`。这只是 FTP 账号，不代表系统 root 权限。
- FTP 为只读服务：允许列目录、查询文件大小及下载，拒绝上传、覆盖；删除、重命名、创建目录等命令不支持。
- 两种登录均禁止 `CDUP` 和含 `..` 的路径；`CWD /` 只返回当前会话的虚拟根目录，不能跨根目录访问。仅日志登录保留 `/log` 别名，`test` 登录下的 `/log` 为存储卡内真实子目录。
- 目标根目录不存在或不可读取时拒绝登录，不自动创建目录或退回其他根目录。重新发送 `USER` 会清除原登录状态与目录选择。
- 文件读取和目录列表逐级打开路径，不跟随符号链接；只能下载普通文件。
- 日志只记录命令名，不记录用户名、密码参数。普通 FTP 不加密，固定密码仅应在可信网络使用。

主机回归测试（使用独立临时目录，不访问设备日志）：

```sh
gcc -Wall -Wextra -pthread $(find src -type d -printf '-I%p ') tests/ftp_access_test.c -o /tmp/ftp_access_test
/tmp/ftp_access_test
```

### FTP 断线重连

- FTP 保持单会话模式：新控制连接到来时，中止旧会话、关闭旧控制连接及数据连接，再接入新连接。新连接必须重新登录；同样适用于旧客户端仍在线的情况。
- 控制命令等待、被动数据连接等待、主动数据连接建立及下载发送均监听新连接，不再等待旧 TCP 连接自行超时。释放旧会话后可重新绑定固定被动端口 `40900`。
- 网络等待上限为 60 秒，控制响应发送等待上限为 1 秒；发送处理短写并抑制 `SIGPIPE`，避免断网时服务被信号终止。

主机重连回归测试（使用临时目录、随机控制端口及本机 `40900` 数据端口，运行时请确保该数据端口空闲）：

```sh
gcc -Wall -Wextra -pthread $(find src -type d -printf '-I%p ') tests/ftp_reconnect_test.c -o /tmp/ftp_reconnect_test
/tmp/ftp_reconnect_test
```

目标板验收：登录后分别在空闲、列目录等待和下载大文件时拔掉网线，再插回并重新连接；无需重启服务，应能重新登录、列目录和下载文件。

### ECU、BCU 与 BMU OTA 文件与版本检查

- ECU OTA 要求文件名包含 `ECU`，且第一个大写 `V` 后紧跟数字，数字不能是 `2`（拒绝 `V2...`）。仅检查文件名，不比较设备当前版本。不符合时在解包或替换程序前以版本不匹配错误拒绝升级。例如 `XC_ECU_V123.tar`、`XC_ECU_V301.tar` 可以通过文件名检查，`XC_ECU_V201.tar`、`XC_ECU_S32K344.bin` 会被拒绝。

- BCU OTA 要求文件名包含 `BCU`，且第一个大写 `V` 后紧跟数字，数字不能是 `3`（拒绝 `V3...`），否则以版本不匹配错误拒绝升级。不读取或比较 BCU 上报的版本高位；未获取到 BCU 版本不影响此检查。
- OCPP 使用 `XC_BCU_` 识别 BCU 升级文件，再由 OTA 预检查统一校验上述命名规则。`XC_BCU_V501.tar`、`XC_BCU_V501.bin`、`XC_BCU_V601.tar` 可以通过文件名检查；`XC_BCU_V301.tar`、`XC_BCU_S32K344.bin` 会被拒绝。直接 `.bin` 文件仍需存在并成功复制到升级准备目录；压缩包仍需通过原有解包检查。
- BMU 支持直接上传 `.bin`，例如 `XC_BMU_V301.bin`，也保留原有压缩包升级。BCU/BMU 在设备升级前统一检查文件：直接 `.bin` 从 `incoming` 复制到 `ready`，不执行 tar 解包，也不要求 `.conf` 或其 MD5 校验；压缩包仍要求原有 `.conf`、文件名及 MD5 校验。BMU 不新增 BCU 的版本限制，`V3...` 仍可使用。
- BMU 文件准备只在批量升级前执行；准备失败则报失败并清理退出，不再对各 BMU 进行无效重试。准备成功后，各 BMU 及其重试复用同一份固件。
- BMU 批量升级进度从 0% 开始，按当前 BMU 已确认传输的固件字节数和整体设备序号逐步更新整数百分比。重试不回退进度；传输阶段最高 99%，仅全部 BMU 升级成功后写入 100%。连接、擦除和等待响应时进度可能暂时不变，不使用虚假计时进度。
- 文件名安全性和其他升级检查保持不变。通过预检查不代表升级成功，后续跳转 BOOT、XCP 连接和传输仍需要设备正常响应。

主机版本与文件准备回归测试（模拟文件操作，不读取固件、不执行实际升级）：

```sh
gcc -Wall -Wextra -ffunction-sections -fdata-sections $(find src -type d -printf '-I%p ') tests/ota_version_test.c -Wl,--gc-sections -o /tmp/ota_version_test
/tmp/ota_version_test
```

主机 BMU 进度回归测试（不连接 CAN、不执行实际升级）：

```sh
gcc -Wall -Wextra -ffunction-sections -fdata-sections $(find src -type d -printf '-I%p ') tests/bmu_progress_test.c -Wl,--gc-sections -o /tmp/bmu_progress_test
/tmp/bmu_progress_test
```

### MA35_ECU 版本更新说明
### v1.00
- MA35_ECU工程初步建立
### v1.01
- 1、增加FTP功能。
- 2、增加日志保存功能。
- 3、增加OTA升级功能。
- 4、增加灯带 、IO等功能模块。
- 5、优化部分功能模块。
### v1.02
- 1、优化IP配置功能。
- 2、修改与BCU时间同步功能。
- 3、优化修改IO检测功能。
### v1.03
- 1、优化相关BCU电池信息透传
### v1.04
- 1、CAN各路功能修改。

### v1.10
- 1、优化U盘异常拔出逻辑。

### v1.11
- 1、优化U盘日志保存功能。
- 2、增加OCPP云平台升级功能（ECU、BCU、BMU）志拉取功能。

### v1.12
- 1、优化空调本地升级和云平台升级功能。
- 2、将ECU升级文件修改为.bz2格式。
- 3、增加websocket日志。

### v1.13
- 1、优化空调主控云端升级流程。

### v1.14
- 1、欧标ECU版本。

### v1.15
- 1、调整模型增加透传电表数据输出。
### v2.00
- 1、整体优化功能，更改到映翰通边缘计算机

### v2.01
- 1、修改了代码结构，实现了新的脚本和服务，都在桌面的deb_work里，后续要变成deb包