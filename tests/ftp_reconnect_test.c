#include <assert.h>
#include <signal.h>
#include <stdarg.h>
#include <sys/wait.h>

static char test_root[] = "/tmp/ecu_ftp_reconnect_XXXXXX";
#define FTP_ROOT_PATH test_root
#include "../src/device_drv/ftp_protocol/ftp_protocol.c"
#include "../src/function_task/ftp_task/ftp_task.c"

Setting_t g_ipsetting;

void log_output(const char *file, size_t filelen, const char *func,
                size_t funclen, long line, const char *format, ...)
{
    (void)file;
    (void)filelen;
    (void)func;
    (void)funclen;
    (void)line;
    (void)format;
}

static double monotonic_seconds(void)
{
    struct timespec now;
    assert(clock_gettime(CLOCK_MONOTONIC, &now) == 0);
    return now.tv_sec + now.tv_nsec / 1000000000.0;
}

static void reply(int fd, const char *prefix)
{
    char buffer[512];
    size_t used = 0;
    do {
        assert(used < sizeof(buffer) - 1);
        assert(recv(fd, buffer + used, 1, 0) == 1);
    } while (buffer[used++] != '\n');
    buffer[used] = '\0';
    if (strncmp(buffer, prefix, strlen(prefix)) != 0) {
        fprintf(stderr, "Expected %s, received %s", prefix, buffer);
        abort();
    }
}

static void command(int fd, const char *text, const char *prefix)
{
    assert(send(fd, text, strlen(text), MSG_NOSIGNAL) == (ssize_t)strlen(text));
    reply(fd, prefix);
}

static int connect_to_port(int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    assert(fd >= 0);
    struct timeval timeout = {.tv_sec = 2};
    assert(setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0);
    assert(setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) == 0);
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
        .sin_port = htons(port)
    };
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static void login(int fd)
{
    command(fd, "USER root\r\n", "331");
    command(fd, "PASS root\r\n", "230");
}

static void passive(int fd)
{
    command(fd, "PASV\r\n", "227");
}

static int replace_session(int old, int port)
{
    double start = monotonic_seconds();
    int next = connect_to_port(port);
    assert(next >= 0);
    reply(next, "220");
    assert(monotonic_seconds() - start < 1.5);
    char buffer[512];
    ssize_t n;
    while ((n = recv(old, buffer, sizeof(buffer), 0)) > 0) {}
    assert(n == 0);
    close(old);
    command(next, "PWD\r\n", "530");
    login(next);
    /* Rebinding the fixed passive port also verifies old listener cleanup. */
    passive(next);
    return next;
}

static void download_small_file(int control)
{
    int data = connect_to_port(DATA_PORT);
    assert(data >= 0);
    command(control, "RETR small.log\r\n", "150");
    char buffer[64] = {0};
    size_t used = 0;
    ssize_t n;
    while ((n = recv(data, buffer + used, sizeof(buffer) - used - 1, 0)) > 0) {
        used += (size_t)n;
        assert(used < sizeof(buffer) - 1);
    }
    assert(n == 0 && strcmp(buffer, "reconnected\n") == 0);
    close(data);
    reply(control, "226");
}

struct send_test {
    int fd;
    size_t length;
};

static void *read_all(void *arg)
{
    struct send_test *test = arg;
    char buffer[4096];
    size_t received = 0;
    while (received < test->length) {
        ssize_t n = recv(test->fd, buffer, sizeof(buffer), 0);
        assert(n > 0);
        for (ssize_t i = 0; i < n; ++i) assert(buffer[i] == 'x');
        received += (size_t)n;
    }
    assert(received == test->length);
    return NULL;
}

static void test_sends(void)
{
    int sockets[2];
    assert(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);
    int size = 4096;
    assert(setsockopt(sockets[0], SOL_SOCKET, SO_SNDBUF, &size, sizeof(size)) == 0);
    size_t length = 2 * 1024 * 1024;
    char *payload = malloc(length);
    assert(payload);
    memset(payload, 'x', length);
    struct send_test test = {.fd = sockets[1], .length = length};
    pthread_t reader;
    assert(pthread_create(&reader, NULL, read_all, &test) == 0);
    assert(send_ftp_data(NULL, sockets[0], payload, length, 2000) == 0);
    assert(pthread_join(reader, NULL) == 0);
    free(payload);

    char buffer[4096] = {0};
    while (send(sockets[0], buffer, sizeof(buffer), MSG_DONTWAIT | MSG_NOSIGNAL) > 0) {}
    assert(errno == EAGAIN || errno == EWOULDBLOCK);
    double start = monotonic_seconds();
    send_response(sockets[0], "220 blocked\r\n");
    double elapsed = monotonic_seconds() - start;
    assert(elapsed >= 0.9 && elapsed < 1.5);
    close(sockets[0]);
    close(sockets[1]);

    /* Default SIGPIPE disposition must not terminate the FTP service. */
    assert(signal(SIGPIPE, SIG_DFL) != SIG_ERR);
    assert(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);
    close(sockets[1]);
    assert(send_ftp_data(NULL, sockets[0], "data", 4, 1000) == -1);
    send_response(sockets[0], "221 Goodbye.\r\n");
    close(sockets[0]);
}

static int replace_pending_active_connection(int control, int port)
{
    int listener = socket(AF_INET, SOCK_STREAM, 0);
    assert(listener >= 0);
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK)
    };
    assert(bind(listener, (struct sockaddr *)&addr, sizeof(addr)) == 0);
    assert(listen(listener, 1) == 0);
    socklen_t len = sizeof(addr);
    assert(getsockname(listener, (struct sockaddr *)&addr, &len) == 0);
    int data_port = ntohs(addr.sin_port);
    /* Fill Linux's backlog+1 queue so PORT waits for its TCP handshake. */
    int first = connect_to_port(data_port);
    int second = connect_to_port(data_port);
    assert(first >= 0 && second >= 0);
    char text[64];
    int length = snprintf(text, sizeof(text), "PORT 127,0,0,1,%d,%d\r\n",
                          data_port / 256, data_port % 256);
    assert(send(control, text, length, MSG_NOSIGNAL) == length);
    usleep(100000);
    char byte;
    assert(recv(control, &byte, 1, MSG_PEEK | MSG_DONTWAIT) == -1);
    assert(errno == EAGAIN || errno == EWOULDBLOCK);
    control = replace_session(control, port);
    close(first);
    close(second);
    close(listener);
    return control;
}

int main(void)
{
    test_sends();
    assert(mkdtemp(test_root));
    char small[512], large[512];
    snprintf(small, sizeof(small), "%s/small.log", test_root);
    snprintf(large, sizeof(large), "%s/large.log", test_root);
    FILE *file = fopen(small, "wb");
    assert(file);
    assert(fputs("reconnected\n", file) >= 0);
    assert(fclose(file) == 0);
    int fd = open(large, O_CREAT | O_WRONLY, 0600);
    assert(fd >= 0);
    assert(ftruncate(fd, 32 * 1024 * 1024) == 0);
    close(fd);

    int reserved = socket(AF_INET, SOCK_STREAM, 0);
    assert(reserved >= 0);
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK)
    };
    assert(bind(reserved, (struct sockaddr *)&addr, sizeof(addr)) == 0);
    socklen_t len = sizeof(addr);
    assert(getsockname(reserved, (struct sockaddr *)&addr, &len) == 0);
    int port = ntohs(addr.sin_port);
    close(reserved);
    g_ipsetting.ip = INADDR_LOOPBACK;
    pid_t server = fork();
    assert(server >= 0);
    if (server == 0) {
        /* Bound test lifetime even if an assertion fails in the parent. */
        alarm(20);
        ftp_service_thread_func(&port);
        _exit(1);
    }

    int control = -1;
    for (int attempt = 0; attempt < 100 && control < 0; ++attempt) {
        control = connect_to_port(port);
        if (control < 0) usleep(10000);
    }
    assert(control >= 0);
    reply(control, "220");
    login(control);
    passive(control);
    /* The old TCP control connection is deliberately left open throughout. */
    for (int i = 0; i < 5; ++i) control = replace_session(control, port);

    command(control, "LIST\r\n", "150");
    control = replace_session(control, port);
    command(control, "NLST\r\n", "150");
    control = replace_session(control, port);
    command(control, "MGET small.log\r\n", "150");
    control = replace_session(control, port);
    control = replace_pending_active_connection(control, port);

    assert(send(control, "RETR small.log\r\n", 16, MSG_NOSIGNAL) == 16);
    usleep(100000);
    control = replace_session(control, port);
    download_small_file(control);
    passive(control);

    const char *transfers[] = {"RETR large.log\r\n", "MGET large.log\r\n"};
    for (size_t i = 0; i < sizeof(transfers) / sizeof(transfers[0]); ++i) {
        int data = connect_to_port(DATA_PORT);
        assert(data >= 0);
        int size = 4096;
        assert(setsockopt(data, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size)) == 0);
        command(control, transfers[i], "150");
        usleep(100000);
        char byte;
        assert(recv(control, &byte, 1, MSG_PEEK | MSG_DONTWAIT) == -1);
        assert(errno == EAGAIN || errno == EWOULDBLOCK);
        control = replace_session(control, port);
        close(data);
        download_small_file(control);
        passive(control);
    }

    int data = connect_to_port(DATA_PORT);
    assert(data >= 0);
    command(control, "RETR large.log\r\n", "150");
    struct linger reset = {.l_onoff = 1, .l_linger = 0};
    assert(setsockopt(data, SOL_SOCKET, SO_LINGER, &reset, sizeof(reset)) == 0);
    close(data);
    reply(control, "426");
    command(control, "PWD\r\n", "257");
    passive(control);
    download_small_file(control);
    command(control, "QUIT\r\n", "221");
    close(control);

    assert(kill(server, SIGTERM) == 0);
    int status;
    assert(waitpid(server, &status, 0) == server);
    assert(WIFSIGNALED(status) && WTERMSIG(status) == SIGTERM);
    assert(unlink(small) == 0);
    assert(unlink(large) == 0);
    assert(rmdir(test_root) == 0);
    puts("FTP reconnect, passive-port reuse, stalled transfer and SIGPIPE tests passed");
    return 0;
}
