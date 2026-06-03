#ifndef DAS_SCI_IO_H
#define DAS_SCI_IO_H

#ifdef __cplusplus
extern "C" {
#endif

//
// Function Prototypes
//
extern int SCI_open(const char * path, unsigned flags, int llv_fd);
extern int SCI_close(int dev_fd);
extern int SCIA_read(int dev_fd, char * buf, unsigned count);
extern int SCIA_write(int dev_fd, const char * buf, unsigned count);
extern int SCIB_read(int dev_fd, char * buf, unsigned count);
extern int SCIB_write(int dev_fd, const char * buf, unsigned count);
extern int SCIC_read(int dev_fd, char * buf, unsigned count);
extern int SCIC_write(int dev_fd, const char * buf, unsigned count);
extern off_t SCI_lseek(int dev_fd, off_t offset, int origin);
extern int SCI_unlink(const char * path);
extern int SCI_rename(const char * old_name, const char * new_name);

#ifdef __cplusplus
}
#endif /* extern "C" */


#endif
