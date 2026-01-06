#ifndef __DBG_H__
#define __DBG_H__

#define  DBG_UART  (&huart1)


void dbg_printf(const char *fmt, ...);

#endif /* __DBG_H__ */