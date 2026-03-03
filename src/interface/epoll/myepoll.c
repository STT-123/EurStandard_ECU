#include "myepoll.h"
#include <sys/epoll.h> // epoll
#include <pthread.h>   // 线程
#include <unistd.h>    // usleep函数
#include <string.h>    // 主要声明 strerror()
#include <errno.h>     // 提供 errno 宏（可选，但通常一起使用）
#include "log.h"       // 日志接口


// 记录一共有多少个epoll事件
static unsigned int epoll_count = 0;
struct epoll_event events[MYMAX_TASTNUM] = {0};
static int my_epollfd = -1; // 用于监听所有能监听的事件
static pthread_t epoll_thread_id = 0;// epoll 初始化

static void *epoll_thread_func(void *arg)
{
    int num;
    int i;
    my_event_data_t event;

    LOG("[EPOLL] epoll_thread_func running ");

    sleep(5); // 此时没有epoll事件

    for (;;)
    {
        // maxevents must be > 0 and must not exceed the events[] capacity.
        num = epoll_wait(my_epollfd, events, MYMAX_TASTNUM, -1);

        if (num < 0)
        {
            //perror("epoll error");
        }
        for (i = 0; i < num; i++) {
            my_event_data_t *ev_data = events[i].data.ptr;
            if (ev_data == NULL) {
                LOG("[EPOLL] NULL event data pointer, skip\n");
                continue;
            }
            // 根据事件类型调用处理函数
            if (events[i].events & (EPOLLPRI | EPOLLIN)) {
                if (ev_data->fun_handle != NULL) {
                    ev_data->fun_handle(ev_data);
                } else {
                    LOG("[EPOLL] NULL event handler, fd=%d\n", ev_data->fd);
                    continue;
                }
            }
            // 无论何种事件类型，都需要清中断
            if (ev_data->fd >= 0) {
                lseek(ev_data->fd, 0, SEEK_SET);
            }
        }
    }

    return NULL;
}

int my_epoll_Init(void)
{
    LOG("[EPOLL] my_epoll_Init ");

    int err = 0;
    my_epollfd = epoll_create(50); // 内核2.6.8+参数无意义，大于零即可

    if (my_epollfd < 0){
        perror("[EPOLL] my epoll create error");
        return -1;
    }

    err = pthread_create(&epoll_thread_id, NULL, &epoll_thread_func, NULL);
    
    if (err != 0){
         LOG("[EPOLL] Create epoll_thread_func Failed, [%s]", strerror(err));
    }
       
    usleep(50 * 1000);
    return 0;
}
/**
 * 回调函数写入ev中
 * int fd: 要监听的文件描述符
 * struct epoll_event * ev: 监听的事件
*/
int my_epoll_addtast(int fd, struct epoll_event *ev)
{
    if (epoll_ctl(my_epollfd, EPOLL_CTL_ADD, fd, ev) < 0)
    {
        perror("epoll_ctl error");
        return -1;
    }
    epoll_count++;

    return 0;
}
/**
 * int fd: 要监听的文件描述符
 * struct epoll_event * ev: 监听的事件
 * 删除epoll监听事件和任务
*/
int my_epoll_deltast(int fd, struct epoll_event *ev)
{
    epoll_ctl(my_epollfd, EPOLL_CTL_DEL, fd, ev);
    epoll_count--;

    return 0;
}
