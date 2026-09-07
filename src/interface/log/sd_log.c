#define _GNU_SOURCE
#include "sd_log.h"
#include <errno.h>
#include <fcntl.h>
#include <mntent.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#ifndef SD_LOG_MOUNT
#define SD_LOG_MOUNT "/mnt/sda"
#endif

_Static_assert(SD_LOG_MAX_SIZE >= SD_LOG_LINE_SIZE, "SD log file must hold one line");
_Static_assert(SD_LOG_BACKUP_COUNT > 0, "SD log needs at least one backup");

static pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
/* 文件 I/O 不持有 queue_mutex，不阻塞业务线程入队。 */
static pthread_mutex_t io_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t wakeup = PTHREAD_COND_INITIALIZER;
static pthread_t worker;
static bool running, stopping, paused;
static char queue[SD_LOG_QUEUE_COUNT][SD_LOG_LINE_SIZE];
static size_t head, count;
static unsigned long dropped;

void sd_log_enqueue(const char *file, long line, const char *format, va_list args)
{
    int saved_errno = errno;
    char text[SD_LOG_LINE_SIZE], stamp[32];
    struct timespec now;
    struct tm local;
    clock_gettime(CLOCK_REALTIME, &now);
    localtime_r(&now.tv_sec, &local);
    strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &local);
    const char *base = strrchr(file, '/');
    int prefix = snprintf(text, sizeof(text), "%s.%03ld [%s:%ld] ",
                          stamp, now.tv_nsec / 1000000, base ? base + 1 : file, line);
    if (prefix < 0 || (size_t)prefix >= sizeof(text) - 2) return;
    errno = saved_errno;
    int n = vsnprintf(text + prefix, sizeof(text) - (size_t)prefix - 1, format, args);
    if (n < 0) return;
    if ((size_t)n >= sizeof(text) - (size_t)prefix - 1) {
        static const char suffix[] = " [truncated]\n";
        memcpy(text + sizeof(text) - sizeof(suffix), suffix, sizeof(suffix));
    } else {
        size_t len = strlen(text);
        if (!len || text[len - 1] != '\n') {
            text[len++] = '\n';
            text[len] = '\0';
        }
    }
    pthread_mutex_lock(&queue_mutex);
    if (running && !stopping) {
        if (count == SD_LOG_QUEUE_COUNT) {
            ++dropped;
        } else {
            strcpy(queue[(head + count) % SD_LOG_QUEUE_COUNT], text);
            ++count;
        }
    }
    pthread_mutex_unlock(&queue_mutex);
}

/* 使用目录 fd 固定挂载对象，避免卸载后路径退回内部文件系统。 */
static int open_log_directory(void)
{
    bool mounted = false;
    FILE *mounts = setmntent("/proc/mounts", "r");
    if (!mounts) return -1;
    struct mntent entry, *ent;
    char buffer[4096];
    while ((ent = getmntent_r(mounts, &entry, buffer, sizeof(buffer)))) {
        if (!strcmp(ent->mnt_dir, SD_LOG_MOUNT) && hasmntopt(ent, "rw")) {
            mounted = true;
            break;
        }
    }
    endmntent(mounts);
    if (!mounted) return -1;
    int root = open(SD_LOG_MOUNT, O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    if (root < 0) return -1;
    struct stat current, parent;
    int parent_fd = openat(root, "..", O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    bool valid = parent_fd >= 0 && fstat(root, &current) == 0 &&
                 fstat(parent_fd, &parent) == 0 && current.st_dev != parent.st_dev;
    if (parent_fd >= 0) close(parent_fd);
    if (!valid || (mkdirat(root, "log", 0755) != 0 && errno != EEXIST)) {
        close(root);
        return -1;
    }
    int dir = openat(root, "log", O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    close(root);
    return dir;
}

static int rotate(int dir)
{
    char old[32], next[32];
    snprintf(old, sizeof(old), "app.log.%d", SD_LOG_BACKUP_COUNT);
    if (unlinkat(dir, old, 0) != 0 && errno != ENOENT) return -1;
    for (int i = SD_LOG_BACKUP_COUNT - 1; i >= 1; --i) {
        snprintf(old, sizeof(old), "app.log.%d", i);
        snprintf(next, sizeof(next), "app.log.%d", i + 1);
        if (renameat(dir, old, dir, next) != 0 && errno != ENOENT) return -1;
    }
    if (renameat(dir, "app.log", dir, "app.log.1") != 0 && errno != ENOENT) return -1;
    return 0;
}

static int append_line(int dir, int *fd, off_t *size, const char *text)
{
    size_t len = strlen(text);
    if (*fd < 0) {
        *fd = openat(dir, "app.log", O_WRONLY | O_APPEND | O_CREAT | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC, 0644);
        if (*fd < 0) return -1;
        struct stat st;
        if (fstat(*fd, &st) != 0 || !S_ISREG(st.st_mode)) return -1;
        *size = st.st_size;
    }
    if (*size + (off_t)len > SD_LOG_MAX_SIZE) {
        if (fsync(*fd) != 0) return -1;
        close(*fd);
        *fd = -1;
        if (rotate(dir) != 0) return -1;
        return append_line(dir, fd, size, text);
    }
    size_t offset = 0;
    while (offset < len) {
        ssize_t n = write(*fd, text + offset, len - offset);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return -1;
        offset += (size_t)n;
        *size += n;
    }
    return 0;
}

/* 每批最多处理一个队列容量，避免持续日志流阻止暂停/退出。 */
static int drain_queue(int dir)
{
    int fd = -1, result = 0;
    off_t size = 0;
    unsigned long lost;
    char text[SD_LOG_LINE_SIZE];
    pthread_mutex_lock(&queue_mutex);
    lost = dropped;
    dropped = 0;
    pthread_mutex_unlock(&queue_mutex);
    if (lost) {
        snprintf(text, sizeof(text), "[SD LOG] dropped %lu messages (queue full or write failure)\n", lost);
        if (append_line(dir, &fd, &size, text) != 0) {
            pthread_mutex_lock(&queue_mutex);
            dropped += lost;
            pthread_mutex_unlock(&queue_mutex);
            result = -1;
        }
    }
    for (size_t i = 0; result == 0 && i < SD_LOG_QUEUE_COUNT; ++i) {
        pthread_mutex_lock(&queue_mutex);
        if (!count) {
            pthread_mutex_unlock(&queue_mutex);
            break;
        }
        strcpy(text, queue[head]);
        pthread_mutex_unlock(&queue_mutex);
        if (append_line(dir, &fd, &size, text) != 0) {
            /* 保留该条供下次重试；部分写入可能留下不完整行。 */
            result = -1;
            break;
        }
        pthread_mutex_lock(&queue_mutex);
        head = (head + 1) % SD_LOG_QUEUE_COUNT;
        --count;
        pthread_mutex_unlock(&queue_mutex);
    }
    if (fd >= 0) {
        if (fsync(fd) != 0) result = -1;
        if (close(fd) != 0) result = -1;
    }
    return result;
}

static void *write_task(void *unused)
{
    (void)unused;
    for (;;) {
        pthread_mutex_lock(&queue_mutex);
        if (!stopping) {
            struct timespec deadline;
            clock_gettime(CLOCK_REALTIME, &deadline);
            ++deadline.tv_sec;
            pthread_cond_timedwait(&wakeup, &queue_mutex, &deadline);
        }
        bool finish = stopping;
        pthread_mutex_unlock(&queue_mutex);
        pthread_mutex_lock(&io_mutex);
        if (!paused) {
            int dir = open_log_directory();
            if (dir >= 0) {
                if (drain_queue(dir) != 0)
                    fprintf(stderr, "[SD LOG] write/flush failed: %s\n", strerror(errno));
                close(dir);
            }
        }
        pthread_mutex_unlock(&io_mutex);
        if (finish) break;
    }
    return NULL;
}

int sd_log_start(void)
{
    pthread_mutex_lock(&queue_mutex);
    int rc = 0;
    if (!running) {
        stopping = false;
        head = count = dropped = 0;
        rc = pthread_create(&worker, NULL, write_task, NULL);
        running = rc == 0;
    }
    pthread_mutex_unlock(&queue_mutex);
    return rc;
}

void sd_log_stop(void)
{
    pthread_mutex_lock(&queue_mutex);
    bool join = running;
    stopping = true;
    pthread_cond_signal(&wakeup);
    pthread_mutex_unlock(&queue_mutex);
    if (join) pthread_join(worker, NULL);
    pthread_mutex_lock(&queue_mutex);
    running = false;
    pthread_mutex_unlock(&queue_mutex);
}

void sd_log_pause(bool value)
{
    pthread_mutex_lock(&io_mutex);
    paused = value;
    pthread_mutex_unlock(&io_mutex);
}
