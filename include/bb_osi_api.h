/**
=====================================================================================================================================================

@fn Name            : bytebeam_osiMessageQueue_t *bytebeam_create_msg_Que(uint32 msg_count, uint32 msg_size)
@b Scope            : Public
@n@n@b Description  : Creates a new message queue.
@param msg_count    : Maximum number of messages in the queue.
@param msg_size     : Size of each message in the queue.
@return Return Value: A pointer to the newly created message queue.

=====================================================================================================================================================
*/
bytebeam_osiMessageQueue_t *bytebeam_create_msg_Que(uint32 msg_count, uint32 msg_size) {
    return nwy_create_msg_Que(msg_count, msg_size);
}

/**
=====================================================================================================================================================

@fn Name            : void bytebeam_delete_msg_que(bytebeam_osiMessageQueue_t *mq)
@b Scope            : Public
@n@n@b Description  : Deletes a message queue.
@param mq           : A pointer to the message queue to be deleted.
@return Return Value: None.

=====================================================================================================================================================
*/
void bytebeam_delete_msg_que(bytebeam_osiMessageQueue_t *mq) {
    nwy_delete_msg_que((nwy_osiMessageQueue_t *)mq);
}

/**
=====================================================================================================================================================

@fn Name            : bool bytebeam_put_msg_que(bytebeam_osiMessageQueue_t *mq, const void *msg, uint32 timeout)
@b Scope            : Public
@n@n@b Description  : Puts a message into the queue.
@param mq           : A pointer to the message queue.
@param msg          : A pointer to the message to be put into the queue.
@param timeout      : Time to wait if the queue is full.
@return Return Value: True if the operation was successful, false otherwise.

=====================================================================================================================================================
*/
bool bytebeam_put_msg_que(bytebeam_osiMessageQueue_t *mq, const void *msg, uint32 timeout) {
    return nwy_put_msg_que((nwy_osiMessageQueue_t *)mq, msg, timeout);
}

/**
=====================================================================================================================================================

@fn Name            : bool bytebeam_get_msg_que(bytebeam_osiMessageQueue_t *mq, const void *msg, uint32 timeout)
@b Scope            : Public
@n@n@b Description  : Gets a message from the queue.
@param mq           : A pointer to the message queue.
@param msg          : A pointer to the place to store the retrieved message.
@param timeout      : Time to wait if the queue is empty.
@return Return Value: True if a message was retrieved, false otherwise.

=====================================================================================================================================================
*/
bool bytebeam_get_msg_que(bytebeam_osiMessageQueue_t *mq, const void *msg, uint32 timeout) {
    return nwy_get_msg_que((nwy_osiMessageQueue_t *)mq, (void *)msg, timeout);
}

/**
=====================================================================================================================================================

@fn Name            : uint32_t bytebeam_get_queue_pendingevent_cnt(bytebeam_osiMessageQueue_t *mq)
@b Scope            : Public
@n@n@b Description  : Gets the number of pending events in a queue.
@param mq           : A pointer to the message queue.
@return Return Value: Number of pending events.

=====================================================================================================================================================
*/
uint32_t bytebeam_get_queue_pendingevent_cnt(bytebeam_osiMessageQueue_t *mq) {
    return nwy_get_queue_pendingevent_cnt((nwy_osiMessageQueue_t *)mq);
}

/**
=====================================================================================================================================================

@fn Name            : uint32_t bytebeam_get_queue_spaceevent_cnt(bytebeam_osiMessageQueue_t *mq)
@b Scope            : Public
@n@n@b Description  : Gets the number of space events in a queue.
@param mq           : A pointer to the message queue.
@return Return Value: Number of space events.

=====================================================================================================================================================
*/
uint32_t bytebeam_get_queue_spaceevent_cnt(bytebeam_osiMessageQueue_t *mq) {
    return nwy_get_queue_spaceevent_cnt((nwy_osiMessageQueue_t *)mq);
}

/**
=====================================================================================================================================================

@fn Name            : bytebeam_osiMutex_t *bytebeam_create_mutex()
@b Scope            : Public
@n@n@b Description  : Creates a new mutex.
@return Return Value: A pointer to the newly created mutex.

=====================================================================================================================================================
*/
bytebeam_osiMutex_t *bytebeam_create_mutex() {
    return nwy_create_mutex();
}

/**
=====================================================================================================================================================

@fn Name            : bool bytebeam_lock_mutex(bytebeam_osiMutex_t *mutex, uint32_t timeout)
@b Scope            : Public
@n@n@b Description  : Locks a mutex.
@param mutex        : A pointer to the mutex.
@param timeout      : Time to wait if the mutex is already locked.
@return Return Value: True if the operation was successful, false otherwise.

=====================================================================================================================================================
*/
bool bytebeam_lock_mutex(bytebeam_osiMutex_t *mutex, uint32_t timeout) {
    return nwy_lock_mutex((nwy_osiMutex_t *)mutex, timeout);
}

/**
=====================================================================================================================================================

@fn Name            : void bytebeam_unlock_mutex(bytebeam_osiMutex_t *mutex)
@b Scope            : Public
@n@n@b Description  : Unlocks a mutex.
@param mutex        : A pointer to the mutex.
@return Return Value: None.

=====================================================================================================================================================
*/
void bytebeam_unlock_mutex(bytebeam_osiMutex_t *mutex) {
    nwy_unlock_mutex((nwy_osiMutex_t *)mutex);
}

/**
=====================================================================================================================================================

@fn Name            : void bytebeam_delete_mutex(bytebeam_osiMutex_t *mutex)
@b Scope            : Public
@n@n@b Description  : Deletes a mutex.
@param mutex        : A pointer to the mutex.
@return Return Value: None.

=====================================================================================================================================================
*/
void bytebeam_delete_mutex(bytebeam_osiMutex_t *mutex) {
    nwy_delete_mutex((nwy_osiMutex_t *)mutex);
}

// Wrapper for Semaphore API
typedef struct 
{
    nwy_osiSemaphore_t *semaphore;
} bytebeam_semaphore_t;

/**
=====================================================================================================================================================

@fn Name            : bytebeam_semaphore_t *bytebeam_semaphore_create(uint32_t max_count, uint32_t init_count)
@b Scope            : Public
@n@n@b Description  : Creates a new semaphore.
@param max_count    : Maximum count for the semaphore.
@param init_count   : Initial count for the semaphore.
@return Return Value: A pointer to the newly created semaphore.

=====================================================================================================================================================
*/
bytebeam_semaphore_t *bytebeam_semaphore_create(uint32_t max_count, uint32_t init_count) 
{
    bytebeam_semaphore_t *new_semaphore = malloc(sizeof(bytebeam_semaphore_t));
    if(new_semaphore != NULL) 
    {
        new_semaphore->semaphore = nwy_semaphore_create(max_count, init_count);
    }
    return new_semaphore;
}

/**
=====================================================================================================================================================

@fn Name            : bool bytebeam_semaphore_acquire(bytebeam_semaphore_t *sem, uint32_t timeout)
@b Scope            : Public
@n@n@b Description  : Acquires a semaphore.
@param sem          : A pointer to the semaphore.
@param timeout      : Time to wait if the semaphore is already locked.
@return Return Value: True if the operation was successful, false otherwise.

=====================================================================================================================================================
*/
bool bytebeam_semaphore_acquire(bytebeam_semaphore_t *sem, uint32_t timeout) 
{
    if(sem != NULL) 
    {
        return nwy_semaphore_acquire(sem->semaphore, timeout);
    } 
    else 
    {
        return false;
    }
}

/**
=====================================================================================================================================================

@fn Name            : void bytebeam_semaphore_release(bytebeam_semaphore_t *sem)
@b Scope            : Public
@n@n@b Description  : Releases a semaphore.
@param sem          : A pointer to the semaphore.
@return Return Value: None.

=====================================================================================================================================================
*/
void bytebeam_semaphore_release(bytebeam_semaphore_t *sem) 
{
    if(sem != NULL) 
    {
        nwy_semahpore_release(sem->semaphore);
    }
}

/**
=====================================================================================================================================================

@fn Name            : void bytebeam_semaphore_delete(bytebeam_semaphore_t *sem)
@b Scope            : Public
@n@n@b Description  : Deletes a semaphore.
@param sem          : A pointer to the semaphore.
@return Return Value: None.

=====================================================================================================================================================
*/
void bytebeam_semaphore_delete(bytebeam_semaphore_t *sem) 
{
    if(sem != NULL) 
    {
        nwy_semahpore_delete(sem->semaphore);
        free(sem);
    }
}

/**
=====================================================================================================================================================

@fn Name            : void bb_usleep(uint32 us)
@b Scope            : Public
@n@n@b Description  : Puts the thread to sleep for a specified number of microseconds.
@param us           : Time in microseconds to sleep.
@return Return Value: None.

=====================================================================================================================================================
*/
void bb_usleep(uint32 us)
{
    nwy_usleep(us);
}

/**
=====================================================================================================================================================

@fn Name            : void bb_sleep(uint32 us)
@b Scope            : Public
@n@n@b Description  : Puts the thread to sleep for a specified number of milliseconds.
@param ms           : Time in milliseconds to sleep.
@return Return Value: None.

=====================================================================================================================================================
*/
void bb_sleep(uint32 us)
{
    nwy_sleep(ms);
}
