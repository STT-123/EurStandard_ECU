
#include "xmodemlisten.h"
#include "xmodemstate.h"
#include "xmodemdata.h"
#include "interface/log/log.h"



void* Lwip_Listen_TASK(void* param)
{
    LOG("[Xmodem] Lwip_Listen_TASK\r\n");

    struct sockaddr_in address, remote;
    socklen_t size;
    memset(&address, 0, sizeof(address));

    otasock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (otasock < 0) {
        LOG("[Xmodem] OTA socket create error\n");
        return NULL;
    }

    int optval = 1;
    if (setsockopt(otasock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
        LOG("[Xmodem] setsockopt failed\n");
        close(otasock);
        return NULL;
    }

    address.sin_family = AF_INET;
    address.sin_port = htons(10030);
    address.sin_addr.s_addr = INADDR_ANY;

    if (bind(otasock, (struct sockaddr *)&address, sizeof(address)) < 0) {
        LOG("[Xmodem] bind error!\n");
        close(otasock);
        return NULL;
    }

    listen(otasock, 1); // 监听

	size = sizeof(remote);
    while (1)
    {
		LOG("[Xmodem] otasock1 before accept: %d\n", otasock1);
		int client_socket = accept(otasock, (struct sockaddr *)&remote, &size);
		int accept_errno = errno;
        if (client_socket >= 0)
        {
            LOG("[Xmodem] accepted client socket = %d\n", client_socket);
            LOG("[Xmodem] Client connected: %s:%d\n",inet_ntoa(remote.sin_addr), ntohs(remote.sin_port));

            pthread_mutex_lock(&task_mutex); // 临界区保护
            if (xmodem_server_stopping)
            {
                pthread_mutex_unlock(&task_mutex);
                close(client_socket);
                continue;
            }

            if (!LwIPTCPDataTaskRunning)
            {
				otasock1 = client_socket;
                setClientConnected(1);
				/* Bind this task to this accepted socket.  Do not let a later
				 * accept change the descriptor used by an existing data task. */
				if (pthread_create(&LwIPTCPDataTaskHandle, NULL, lwip_data_TASK,
							   (void *)(intptr_t)client_socket) == 0) {
                    LwIPTCPDataTaskRunning = 1;
                    LOG("[Xmodem] create lwip_data_TASK success\n");
                } else {
                    LOG("[Xmodem] create lwip_data_TASK failed\n");
					close(otasock1);
					otasock1 = -1;
					setClientConnected(0);
                }
            }
            else
            {
				LOG("[Xmodem] lwip_data_TASK already running, reject client socket %d!\n", client_socket);
				close(client_socket);
            }
            pthread_mutex_unlock(&task_mutex);
        }
		else
		{
			int stopping;
			pthread_mutex_lock(&task_mutex);
			stopping = xmodem_server_stopping;
			pthread_mutex_unlock(&task_mutex);
			if (!stopping)
			{
				LOG("[Xmodem] accept failed: errno=%d(%s)\n",
					accept_errno, strerror(accept_errno));
			}
		}

        // 防止任务卡死
        usleep(100 * 1000); // 等价于 vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    LOG("[Xmodem] Lwip_Listen_TASK exited\n");
    return NULL;
}
