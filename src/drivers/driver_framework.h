/*
 * Driver Interface
 *
 * Provides a standardized POSIX API (open/close/read/write/ioctl)
 * for embedded drivers
 */

#ifndef DRIVER_FRAMEWORK_H
#define DRIVER_FRAMEWORK_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <sys/types.h>

/* File descriptor type */
typedef int driver_fd_t;

/* Invalid file descriptor */
#define DRIVER_FD_INVALID (-1)

/* Maximum number of open driver instances per driver type */
#define MAX_DRIVER_INSTANCES 4

/*
 * Standard Driver Error Codes
 */
#define DRIVER_OK           0       /* Success */
#define DRIVER_ERR_INVAL    -EINVAL /* Invalid argument */
#define DRIVER_ERR_NODEV    -ENODEV /* Device not found/ready */
#define DRIVER_ERR_NOMEM    -ENOMEM /* Out of memory/instances */
#define DRIVER_ERR_BUSY     -EBUSY  /* Device busy */
#define DRIVER_ERR_IO       -EIO    /* I/O error */
#define DRIVER_ERR_BADF     -EBADF  /* Bad file descriptor */
#define DRIVER_ERR_AGAIN    -EAGAIN /* Try again */
#define DRIVER_ERR_NOTSUP   -ENOTSUP /* Operation not supported */

/*
 * Generic ioctl Commands
 * Driver-specific commands should start from 0x1000
 */
#define DRIVER_IOCTL_GET_STATUS     0x0001  /* Get driver status */
#define DRIVER_IOCTL_RESET          0x0002  /* Reset driver */
#define DRIVER_IOCTL_GET_INFO       0x0003  /* Get driver info */

/*
 * Driver State - Tracks instance lifecycle
 */
typedef enum {
    DRIVER_STATE_CLOSED = 0,    /* Not initialized */
    DRIVER_STATE_OPEN,          /* Open and ready */
    DRIVER_STATE_ERROR,         /* Error state */
} driver_state_t;

/*
 * Driver Instance Base Structure
 * Each driver should embed this at the start of their private state struct
 */
typedef struct {
    driver_state_t state;       /* Current state */
    uint32_t flags;             /* Open flags */
    struct k_mutex lock;        /* Mutex for thread-safe access */
} driver_instance_t;

/*
 * Driver Operations - Function pointer table
 * Each driver implements these operations
 */
typedef struct {
    driver_fd_t (*open)(uint32_t flags);
    int (*close)(driver_fd_t fd);
    ssize_t (*read)(driver_fd_t fd, void *buf, size_t count);
    ssize_t (*write)(driver_fd_t fd, const void *buf, size_t count);
    int (*ioctl)(driver_fd_t fd, unsigned int cmd, void *arg);
} driver_ops_t;

/*
 * Helper Macros for Driver Implementation
 */

/* Initialize driver instance base */
#define DRIVER_INSTANCE_INIT(inst) \
    do { \
        (inst)->state = DRIVER_STATE_CLOSED; \
        (inst)->flags = 0; \
        k_mutex_init(&(inst)->lock); \
    } while (0)

/* Lock driver instance */
#define DRIVER_LOCK(inst) k_mutex_lock(&(inst)->lock, K_FOREVER)

/* Unlock driver instance */
#define DRIVER_UNLOCK(inst) k_mutex_unlock(&(inst)->lock)

/* Validate file descriptor */
#define DRIVER_FD_VALID(fd, max) ((fd) >= 0 && (fd) < (max))

/* Check if driver instance is open */
#define DRIVER_IS_OPEN(inst) ((inst)->state == DRIVER_STATE_OPEN)

/*
 * File Descriptor Allocation Helper
 *
 * Manages fd allocation from a pool of driver instances.
 * Returns fd index (>=0) on success, DRIVER_FD_INVALID on failure.
 */
static inline driver_fd_t driver_fd_alloc(driver_instance_t **instances,
                                          size_t max_instances)
{
    for (size_t i = 0; i < max_instances; i++) {
        if (instances[i] && instances[i]->state == DRIVER_STATE_CLOSED) {
            return (driver_fd_t)i;
        }
    }
    return DRIVER_FD_INVALID;
}

/*
 * File Descriptor Validation Helper
 *
 * Validates fd and returns instance pointer.
 * Returns instance on success, NULL on error.
 */
static inline driver_instance_t *driver_fd_to_instance(driver_fd_t fd,
                                                       driver_instance_t **instances,
                                                       size_t max_instances)
{
    if (!DRIVER_FD_VALID(fd, max_instances)) {
        return NULL;
    }

    driver_instance_t *inst = instances[fd];
    if (!inst || !DRIVER_IS_OPEN(inst)) {
        return NULL;
    }

    return inst;
}

#endif /* DRIVER_FRAMEWORK_H */
