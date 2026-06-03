//
// Included Files
//
#include <stdio.h>
#include <file.h>
#include <stdint.h>
#include <stdbool.h>

#include "F28x_Project.h"
#include "sci_io.h"

//
// Defines
//

//
// Globals
//
uint16_t deviceOpen = 0;

//
// Functions
//

//
// SCI_open -
//
int SCI_open(const char * path, unsigned flags, int llv_fd)
{
    if(deviceOpen)
    {
        return (-1);
    }
    else
    {
        deviceOpen = 1;
        return (1);
    }
}

//
// SCI_close -
//
int SCI_close(int dev_fd)
{
    if((dev_fd != 1) || (!deviceOpen))
    {
        return (-1);
    }
    else
    {
        deviceOpen = 0;
        return (0);
    }
}

//
// SCIA_read -
//
int SCIA_read(int dev_fd, char * buf, unsigned count)
{
    uint16_t readCount = 0;
    uint16_t * bufPtr = (uint16_t *) buf;

    if(count == 0)
    {
        return (0);
    }

    while((readCount < count) && SciaRegs.SCIRXST.bit.RXRDY)
    {
        *bufPtr = SciaRegs.SCIRXBUF.all;
        readCount++;
        bufPtr++;
    }

    return (readCount);
}

//
// SCIA_write -
//
int SCIA_write(int dev_fd, const char * buf, unsigned count)
{
    uint16_t writeCount = 0;
    uint16_t * bufPtr = (uint16_t *) buf;

    if(count == 0)
    {
        return (0);
    }

    while(writeCount < count)
    {
        while(!SciaRegs.SCICTL2.bit.TXRDY);
        SciaRegs.SCITXBUF.all = *bufPtr;
        writeCount++;
        bufPtr++;
    }

    return (writeCount);
}

//
// SCIB_read -
//
int SCIB_read(int dev_fd, char * buf, unsigned count)
{
    uint16_t readCount = 0;
    uint16_t * bufPtr = (uint16_t *) buf;

    if(count == 0)
    {
        return (0);
    }

    while((readCount < count) && ScibRegs.SCIRXST.bit.RXRDY)
    {
        *bufPtr = ScibRegs.SCIRXBUF.all;
        readCount++;
        bufPtr++;
    }

    return (readCount);
}

//
// SCIB_write -
//
int SCIB_write(int dev_fd, const char * buf, unsigned count)
{
    uint16_t writeCount = 0;
    uint16_t * bufPtr = (uint16_t *) buf;

    if(count == 0)
    {
        return (0);
    }

    while(writeCount < count)
    {
        while(!ScibRegs.SCICTL2.bit.TXRDY);
        ScibRegs.SCITXBUF.all = *bufPtr;
        writeCount++;
        bufPtr++;
    }

    return (writeCount);
}
//
// SCIC_read -
//

int SCIC_read(int dev_fd, char * buf, unsigned count)
{
    uint16_t readCount = 0;
    uint16_t * bufPtr = (uint16_t *) buf;

    if(count == 0)
    {
        return (0);
    }

    while((readCount < count) && ScicRegs.SCIRXST.bit.RXRDY)
    {
        *bufPtr = ScicRegs.SCIRXBUF.all;
        readCount++;
        bufPtr++;
    }

    return (readCount);
}

//
// SCIC_write -
//
int SCIC_write(int dev_fd, const char * buf, unsigned count)
{
    uint16_t writeCount = 0;
    uint16_t * bufPtr = (uint16_t *) buf;

    if(count == 0)
    {
        return (0);
    }

    while(writeCount < count)
    {
        while(!ScicRegs.SCICTL2.bit.TXRDY);
        ScicRegs.SCITXBUF.all = *bufPtr;
        writeCount++;
        bufPtr++;
    }

    return (writeCount);
}

//
// SCI_lseek -
//
off_t SCI_lseek(int dev_fd, off_t offset, int origin)
{
    return (0);
}

//
// SCI_unlink -
//
int SCI_unlink(const char * path)
{
    return (0);
}

//
// SCI_rename -
//
int SCI_rename(const char * old_name, const char * new_name)
{
    return (0);
}

//
// End of File
//
