#include <stdio.h>
#include <stdint.h>

extern int send_char_dbg(int c) ;

size_t __write(int handle, const unsigned char * buffer, size_t size)
{
    int i;
////    UNUSED_PARAMETER(handle);
    for (i = 0; i < size; i++)
    {
////     UNUSED_VARIABLE(app_uart_put(*buffer++));
///     my_uart_tx(*buffer++);
     send_char_dbg(*buffer++);
     
    }
    return size;
}
extern int get_byte_dbg(void) ;

size_t __read(int handle, unsigned char * buffer, size_t size)
{
 ///   UNUSED_PARAMETER(handle);
 ////   UNUSED_PARAMETER(size);
  *buffer=get_byte_dbg();
#if 0    
    while (app_uart_get((uint8_t *)buffer) == NRF_ERROR_NOT_FOUND)
    {
        // No implementation needed.
    }
#endif
    return 1;
}
extern int rdy_rx_dbg(void) ;

int __data_rdy( int handle)
{
////return platform_stdio_rdy();
return rdy_rx_dbg() ;
}

long __lseek(int handle, long offset, int whence)
{
    return -1;
}
int __close(int handle)
{
    return 0;
}
int remove(const char *ch)
{
    return 0;
}


