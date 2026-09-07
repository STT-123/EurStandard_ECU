/* gcc -Wall -Wextra -Werror -pthread tests/sd_log_test.c src/interface/log/log.c -o /tmp/sd_log_test */
#define SD_LOG_MOUNT "/tmp/sd-log-unmounted-test"
#include "../src/interface/log/sd_log.c"
#include "../src/interface/log/log.h"
#include <assert.h>
#include <stdlib.h>

static int zlog_calls;
int zlog_init(const char *path) { (void)path; return 0; }
void zlog_fini(void) {}
zlog_category_t *zlog_get_category(const char *name) { (void)name; return NULL; }
void vzlog(zlog_category_t *category, const char *file, size_t filelen,
           const char *func, size_t funclen, long line, int level,
           const char *format, va_list args)
{
    char text[128];
    assert(category && filelen == strlen(file) && funclen == strlen(func));
    assert(line > 0 && level == ZLOG_LEVEL_INFO);
    vsnprintf(text, sizeof(text), format, args);
    assert(!strcmp(text, "value=42 text=hello"));
    ++zlog_calls;
}

static void enqueue(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    sd_log_enqueue("test.c", 123, format, args);
    va_end(args);
}

static void read_file(int dir, const char *name, char *buf, size_t len)
{
    int fd = openat(dir, name, O_RDONLY);
    assert(fd >= 0);
    ssize_t n = read(fd, buf, len - 1);
    assert(n >= 0);
    buf[n] = 0;
    close(fd);
}

static void *producer(void *unused)
{
    (void)unused;
    for (int i = 0; i < 1000; ++i) enqueue("message %d", i);
    return NULL;
}

int main(void)
{
    assert(open_log_directory() == -1); /* No writes to an unmounted path. */
    char path[] = "/tmp/sd-log-test-XXXXXX";
    assert(mkdtemp(path));
    int dir = open(path, O_RDONLY | O_DIRECTORY);
    assert(dir >= 0);
    running = true; /* Deterministic queue tests, without worker. */
    log_record = (zlog_category_t *)1;
    log_printf = (zlog_category_t *)2;
    errno = EIO;
    LOG("value=%d text=%s", 42, "hello");
    assert(errno == EIO && zlog_calls == 2 && count == 1);
    assert(strstr(queue[head], "value=42 text=hello\n"));
    assert(drain_queue(-1) == -1 && count == 1); /* Failed write retains entry. */
    assert(drain_queue(dir) == 0 && count == 0);
    char content[SD_LOG_LINE_SIZE];
    read_file(dir, "app.log", content, sizeof(content));
    assert(strstr(content, "value=42 text=hello\n"));

    char large[SD_LOG_LINE_SIZE * 2];
    memset(large, 'x', sizeof(large) - 1);
    large[sizeof(large) - 1] = 0;
    enqueue("%s", large);
    assert(strlen(queue[head]) == SD_LOG_LINE_SIZE - 1);
    assert(strstr(queue[head], " [truncated]\n"));
    assert(drain_queue(dir) == 0);

    pthread_t threads[4];
    for (int i = 0; i < 4; ++i) assert(!pthread_create(&threads[i], NULL, producer, NULL));
    for (int i = 0; i < 4; ++i) pthread_join(threads[i], NULL);
    assert(count == SD_LOG_QUEUE_COUNT && dropped == 4000 - SD_LOG_QUEUE_COUNT);
    assert(drain_queue(dir) == 0 && count == 0 && dropped == 0);

    /* Exercise actual 10 MiB boundary using sparse files; check backup order/count. */
    for (int i = 0; i < 7; ++i) {
        int fd = openat(dir, "app.log", O_WRONLY | O_TRUNC);
        assert(fd >= 0);
        char marker = 'A' + i;
        assert(write(fd, &marker, 1) == 1);
        assert(ftruncate(fd, SD_LOG_MAX_SIZE) == 0);
        close(fd);
        enqueue("after rotation %d", i);
        assert(drain_queue(dir) == 0);
    }
    for (int i = 1; i <= SD_LOG_BACKUP_COUNT; ++i) {
        char name[32];
        snprintf(name, sizeof(name), "app.log.%d", i);
        read_file(dir, name, content, sizeof(content));
        assert(content[0] == 'H' - i);
        struct stat st;
        assert(fstatat(dir, name, &st, 0) == 0 && st.st_size == SD_LOG_MAX_SIZE);
    }
    assert(faccessat(dir, "app.log.6", F_OK, 0) != 0);
    read_file(dir, "app.log", content, sizeof(content));
    assert(strstr(content, "after rotation 6"));
    running = false;
    sd_log_pause(true);
    assert(sd_log_start() == 0);
    enqueue("paused");
    sleep(2);
    pthread_mutex_lock(&queue_mutex);
    assert(count == 1);
    pthread_mutex_unlock(&queue_mutex);
    sd_log_stop();
    sd_log_stop();
    sd_log_pause(false);

    unlinkat(dir, "app.log", 0);
    for (int i = 1; i <= SD_LOG_BACKUP_COUNT; ++i) {
        char name[32];
        snprintf(name, sizeof(name), "app.log.%d", i);
        unlinkat(dir, name, 0);
    }
    close(dir);
    assert(rmdir(path) == 0);
    puts("SD log tests passed: fan-out, varargs, errno, retry, truncation, concurrent queue, rotation, pause/stop");
    return 0;
}
