
#ifndef __UART_OPT_H__
#define __UART_OPT_H__

#include "hw_opt.h"

#define RS485_FILE_NAME "/dev/ttyHSU1" 

class CUartOpt
{
public:
	CUartOpt();
	~CUartOpt();
	static CUartOpt* instance();
	int DataCommuni(uchar *in_buf, uchar in_len, uchar *out_buf, uchar out_len);
	int Communi_init(void);
	int Communi_destroy(void);
private:
	
	int uart_open(int fd, char* port);
	void uart_close(int fd);
	int uart_set(int fd, int speed, int flow_ctrl, int databits, int stopbits, int parity);
	int uart_init(void);
	int uart_recv(int fd, uchar *rcv_buf);
	int uart_send(int fd, char *send_buf, int data_len);

private:
	int rs485_fd;
	bool isRs485Open;

	//´®¿Ú·¢ËÍ»¥³âËø
	pthread_mutex_t m_uartMutex;
	pthread_mutexattr_t m_uartMutexAttr;
};
#endif

