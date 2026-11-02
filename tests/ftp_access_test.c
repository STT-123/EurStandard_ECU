#include <assert.h>
#include <stdarg.h>
#include <pthread.h>
#include <time.h>

static char test_root[] = "/tmp/ecu_ftp_test_XXXXXX";
static char sd_root[] = "/tmp/ecu_ftp_sd_test_XXXXXX";
#define FTP_ROOT_PATH test_root
#define FTP_TEST_ROOT_PATH sd_root
#include "../src/device_drv/ftp_protocol/ftp_protocol.c"

Setting_t g_ipsetting;

void log_output(const char *file, size_t filelen, const char *func,
                size_t funclen, long line, const char *format, ...)
{
    (void)file;
    (void)filelen;
    (void)func;
    (void)funclen;
    (void)line;
    /* Command logging must not include the supplied credentials. */
    if (strcmp(format, "Received FTP command: %s\n") == 0) {
        va_list args;
        va_start(args, format);
        assert(strchr(va_arg(args, const char *), ' ') == NULL);
        va_end(args);
    }
}

static void *serve(void *arg)
{
    handle_ftp_commands(arg);
    return NULL;
}

static void reply(int fd, const char *prefix)
{
    char buffer[4096];
    ssize_t n = recv(fd, buffer, sizeof(buffer) - 1, 0);
    assert(n > 0);
    buffer[n] = '\0';
    assert(strncmp(buffer, prefix, strlen(prefix)) == 0);
}

static void command(int fd, const char *text, const char *prefix)
{
    assert(send(fd, text, strlen(text), 0) == (ssize_t)strlen(text));
    reply(fd, prefix);
}

static int data_connection(FTPState *state)
{
    state->data_sock = socket(AF_INET, SOCK_STREAM, 0);
    assert(state->data_sock >= 0);
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    assert(bind(state->data_sock, (struct sockaddr *)&addr, sizeof(addr)) == 0);
    assert(listen(state->data_sock, 1) == 0);
    socklen_t len = sizeof(addr);
    assert(getsockname(state->data_sock, (struct sockaddr *)&addr, &len) == 0);
    int client = socket(AF_INET, SOCK_STREAM, 0);
    assert(client >= 0);
    assert(connect(client, (struct sockaddr *)&addr, len) == 0);
    return client;
}

static void run_suite(const char *root, const char *password, const char *other_root)
{
    char path[512], sub[512], link[512], dirlink[512], fifo[512], logdir[512];
    char crosslink[512], login[64];
    snprintf(login, sizeof(login), "PASS %s\r\n", password);
    snprintf(path, sizeof(path), "%s/example.log", root);
    snprintf(sub, sizeof(sub), "%s/sub", root);
    snprintf(link, sizeof(link), "%s/outside", root);
    snprintf(dirlink, sizeof(dirlink), "%s/outdir", root);
    snprintf(fifo, sizeof(fifo), "%s/pipe", root);
    snprintf(logdir, sizeof(logdir), "%s/log", root);
    snprintf(crosslink, sizeof(crosslink), "%s/other", root);
    FILE *file = fopen(path, "wb");
    assert(file);
    assert(fputs("test log\n", file) >= 0);
    assert(fclose(file) == 0);
    assert(mkdir(sub, 0700) == 0);
    assert(mkdir(logdir, 0700) == 0);
    assert(symlink(other_root, crosslink) == 0);
    assert(symlink("/etc/passwd", link) == 0);
    assert(symlink("/etc", dirlink) == 0);
    assert(mkfifo(fifo, 0600) == 0);

    int sockets[2];
    assert(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);
    struct timeval timeout = {.tv_sec = 3};
    assert(setsockopt(sockets[0], SOL_SOCKET, SO_RCVTIMEO,
                      &timeout, sizeof(timeout)) == 0);
    FTPState state = {0};
    init_ftp_state(&state);
    state.control_sock = sockets[1];
    pthread_t thread;
    assert(pthread_create(&thread, NULL, serve, &state) == 0);

    command(sockets[0], "SIZE example.log\r\n", "530");
    command(sockets[0], "PASS root\r\n", "530");
    command(sockets[0], "USER anonymous\r\n", "530");
    command(sockets[0], "PASS root\r\n", "530");
    command(sockets[0], "USER anonymous\r\n", "530");
    command(sockets[0], "PASS test\r\n", "530");
    command(sockets[0], "USER root\r\n", "331");
    command(sockets[0], "PASS wrong\r\n", "530");
    command(sockets[0], "PASS root\r\n", "530");
    command(sockets[0], "USER root\r\n", "331");
    command(sockets[0], "PASS\r\n", "530");
    command(sockets[0], "USER root\r\n", "331");
    command(sockets[0], login, "230");
    command(sockets[0], "PWD\r\n", "257 \"/\"");
    command(sockets[0], "SIZE example.log\r\n", "213 9");
    command(sockets[0], "STOR example.log\r\n", "550");
    command(sockets[0], "STOR new.log\r\n", "550");
    const char *writes[] = {"DELE example.log\r\n", "APPE example.log\r\n",
        "MKD newdir\r\n", "RMD sub\r\n", "RNFR example.log\r\n",
        "RNTO renamed.log\r\n", "STOU\r\n", "SITE CHMOD 777 example.log\r\n"};
    for (size_t i = 0; i < sizeof(writes) / sizeof(writes[0]); ++i)
        command(sockets[0], writes[i], "502");
    command(sockets[0], "CDUP\r\n", "550");
    command(sockets[0], "CWD ..\r\n", "550");
    command(sockets[0], "CWD /../\r\n", "550");
    command(sockets[0], "CWD outdir\r\n", "550");
    command(sockets[0], "CWD other\r\n", "550");
    command(sockets[0], "SIZE ../example.log\r\n", "550");
    command(sockets[0], "SIZE outside\r\n", "550");
    command(sockets[0], "SIZE outdir/passwd\r\n", "550");
    command(sockets[0], "SIZE pipe\r\n", "550");
    command(sockets[0], "SIZE sub\r\n", "550");
    command(sockets[0], "CWD sub\r\n", "250");
    command(sockets[0], "CDUP\r\n", "550");
    command(sockets[0], "CWD /\r\n", "250");
    command(sockets[0], "CWD log\r\n", "250");
    command(sockets[0], "PWD\r\n",
            strcmp(password, "root") == 0 ? "257 \"/\"" : "257 \"/log\"");
    command(sockets[0], "CWD /\r\n", "250");
    command(sockets[0], "USER wrong\r\n", "530");
    command(sockets[0], "SIZE example.log\r\n", "530");
    command(sockets[0], "USER root\r\n", "331");
    command(sockets[0], login, "230");
    command(sockets[0], "USER root\r\n", "331");
    command(sockets[0], "SIZE example.log\r\n", "530");
    command(sockets[0], strcmp(password, "root") == 0 ?
            "PASS test\r\n" : "PASS root\r\n", "230");
    command(sockets[0], "PWD\r\n", "257 \"/\"");
    command(sockets[0], "SIZE example.log\r\n", "550");
    command(sockets[0], "USER root\r\n", "331");
    command(sockets[0], login, "230");
    command(sockets[0], "SIZE example.log\r\n", "213 9");
    assert(shutdown(sockets[0], SHUT_WR) == 0);
    assert(pthread_join(thread, NULL) == 0);
    close(sockets[0]);

    /* Exercise actual downloads without requiring port 21 or the real log tree. */
    assert(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);
    init_ftp_state(&state);
    state.control_sock = sockets[1];
    state.logged_in = 1;
    state.root_path = root;
    strcpy(state.path, root);
    int client = data_connection(&state);
    handle_retr_command(&state, "example.log");
    char buffer[64] = {0};
    assert(recv(client, buffer, sizeof(buffer), 0) == 9);
    assert(strcmp(buffer, "test log\n") == 0);
    close(client);
    client = data_connection(&state);
    handle_mget_command(&state, "example.log");
    memset(buffer, 0, sizeof(buffer));
    assert(recv(client, buffer, sizeof(buffer), 0) == 9);
    assert(strcmp(buffer, "test log\n") == 0);
    close(client);
    client = data_connection(&state);
    handle_list_command(&state, NULL);
    char listing[4096] = {0};
    size_t used = 0;
    ssize_t n;
    while ((n = recv(client, listing + used, sizeof(listing) - used - 1, 0)) > 0) {
        used += (size_t)n;
        assert(used < sizeof(listing) - 1);
    }
    assert(n == 0);
    assert(strstr(listing, "example.log"));
    close(client);
    close(sockets[0]);
    cleanup_ftp_state(&state);

    state.root_path = root;
    assert(!open_readonly_file(&state, link));
    assert(!open_readonly_file(&state, fifo));
    assert(!open_readonly_file(&state, sub));
    assert(!open_readonly_file(&state, other_root));
    file = fopen(path, "rb");
    assert(file);
    memset(buffer, 0, sizeof(buffer));
    assert(fread(buffer, 1, sizeof(buffer), file) == 9);
    assert(strcmp(buffer, "test log\n") == 0);
    fclose(file);
    char newpath[512];
    snprintf(newpath, sizeof(newpath), "%s/new.log", root);
    assert(access(newpath, F_OK) == -1 && errno == ENOENT);
    assert(unlink(path) == 0);
    assert(unlink(link) == 0);
    assert(unlink(dirlink) == 0);
    assert(unlink(fifo) == 0);
    assert(unlink(crosslink) == 0);
    assert(rmdir(sub) == 0);
    assert(rmdir(logdir) == 0);
}

int main(void)
{
    assert(mkdtemp(test_root));
    assert(mkdtemp(sd_root));
    run_suite(test_root, "root", sd_root);
    run_suite(sd_root, "test", test_root);
    assert(rmdir(sd_root) == 0);
    /* A missing SD root must fail login rather than fall back to logs. */
    int sockets[2];
    assert(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);
    FTPState state = {0};
    init_ftp_state(&state);
    state.control_sock = sockets[1];
    handle_user_command(&state, "root");
    reply(sockets[0], "331");
    handle_pass_command(&state, "test");
    reply(sockets[0], "550");
    assert(!state.logged_in && !state.root_path && !state.path[0]);
    cleanup_ftp_state(&state);
    close(sockets[0]);
    assert(rmdir(test_root) == 0);
    puts("FTP dual-root authentication, read-only and isolation tests passed");
    return 0;
}
