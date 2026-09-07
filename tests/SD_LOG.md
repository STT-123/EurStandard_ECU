# SD 日志副本

仅现有 `LOG(...)` 内容会额外写入 `/mnt/sda/log/app.log`，不需要修改
`/opt/xcharge/zlog.conf`。`LOG_CSV`、直接 printf 和原有 CAN 存储路径保持原样。
SD 行包含调用时的本地时间（毫秒）、源文件名、行号和日志正文。

参数在 `src/interface/log/sd_log.h`：

- `SD_LOG_MAX_SIZE`：单文件 10 MiB。
- `SD_LOG_BACKUP_COUNT`：5 个历史文件，`.1` 最新，`.5` 最旧。
- 总文件大小约 60 MiB，只轮转 `app.log*`，不删除目录内其他日志。
- `SD_LOG_QUEUE_COUNT`：256 条，队列约 1 MiB。
- `SD_LOG_LINE_SIZE`：每条最多 4095 字节，超长部分以 `[truncated]` 标明。

后台约每秒处理最多 256 条并 fsync，业务线程不执行 SD 文件 I/O。
未挂载时保留有限队列，满后丢弃新副本并累计数量，恢复后写入丢失提示。
写入失败保留当前条重试，部分写入可能产生不完整或重复记录。
突然断电、SIGKILL、崩溃或 fsync 失败可能丢失尚未持久化的日志。
SIGINT/SIGTERM 由主线程执行一次最后刷新；正常 exit 使用 atexit 清理。
SD I/O 在内核中卡住时，业务日志入队仍可继续，但格式化和退出会等待 I/O 完成。

格式化入口 SD_Initialize 会先暂停副本并等待文件关闭，再卸载和格式化。
恢复后重新检查挂载并打开文件。此模块不负责挂载 SD，沿用原有挂载流程。

## 主机验证

```sh
gcc -Wall -Wextra -Werror -pthread tests/sd_log_test.c src/interface/log/log.c -o /tmp/sd_log_test
/tmp/sd_log_test
make -j4
```

测试覆盖原有 zlog 双输出和参数、errno 保留、队列并发/溢出、超长行、
写入失败重试、实际 10 MiB 边界和历史文件保留、暂停/停止以及未挂载拒写。

## 上板验收

1. 正常挂载 SD 后启动程序，检查 app.log 是否包含现有 LOG 的正文。
2. 触发足够日志，确认单文件不超过 10 MiB，最多保留 app.log.1 到 app.log.5。
3. 无卡启动，确认内部日志继续记录且不会在未挂载目录创建日志。
4. SD 恢复后检查副本恢复写入；队列满时应出现 dropped 提示。
5. 执行原有 SD 格式化命令，确认没有因本模块持有文件而卸载失败，格式化后恢复写入。
6. SIGTERM 停止程序，检查退出前已入队日志是否刷新。

真实拔卡、只读/满卡和硬件 I/O 故障仍需在设备上验证。
