#ifndef __DBG_H__
#define __DBG_H__

#ifdef COM_DBG_PRINTF
#include "printf.h"
#define  dbg_printf  printf
#endif
#define  DBG_UART  (&huart1)


#endif /* __DBG_H__ */