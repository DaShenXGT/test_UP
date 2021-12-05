#ifndef	_TESTMODE_H
#define	_TESTMODE_H

#define MODEL_CODE		0x01002
#define	SOFTWARE_VER	0x10
#define	VERSION_CODE	0x10

#define BAUD19			0xD0//0xD0
#define SPEED_ADD_BASE	0x5300
#define W_T_ADD_BASE	0x5320
#define O_T_ADD_BASE	0x5340
#define O_L_ADD_BASE	0x5360
#define O_P_ADD_BASE	0x5380

#define	MODEL_CONST		50

#define	TXBUF_SIZE		100
#define	RXBUF_SIZE		100


#define digit(c)		((c) >= '0' && (c) <= '9')
#define hex(c)			(digit(c) || ((c) >= 'A' && (c) <= 'F'))

#define incp_rxd(ptr, n) \
	do { \
		if ((ptr += n) >= rxd_buf + RXBUF_SIZE) \
			ptr -= RXBUF_SIZE; \
	} while (0)
#define decp_rxd(ptr, n) \
	do { \
		if ((ptr -= n) < rxd_buf) \
			ptr += RXBUF_SIZE; \
	} while (0)
#define incp_txd(ptr, n) \
	do { \
		if ((ptr += n) >= txd_buf + TXBUF_SIZE) \
			ptr -= TXBUF_SIZE; \
	} while (0)
#define decp_txd(ptr, n) \
	do { \
		if ((ptr -= n) < txd_buf) \
			ptr += TXBUF_SIZE; \
	} while (0)


#define rxd_overflow() \
	(rxd_head == rxd_tail - 1 || rxd_head == rxd_tail + RXBUF_SIZE - 1)
#define txd_overflow() \
	(txd_head == txd_tail - 1 || txd_head == txd_tail + TXBUF_SIZE - 1)
/*
struct	uart_parameter
{
	unsigned char uart_txdata[TXBUF_SIZE];
	unsigned int  txd_crc;
	unsigned char txd_star;
	unsigned char uart_rxdata[RXBUF_SIZE];
	unsigned int  rxd_crc;
	unsigned char rxd_over;
};
*/
void uart_model_init(void);
void config_uart(void);
void	test_handle(void);
#endif