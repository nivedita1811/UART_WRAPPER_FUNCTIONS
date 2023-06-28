// Create a Message Queue
bytebeam_osiMessageQueue_t *bytebeam_create_msg_Que(uint32 msg_count, uint32 msg_size) {
    return nwy_create_msg_Que(msg_count, msg_size);
}

// Delete a Message Queue
void bytebeam_delete_msg_que(bytebeam_osiMessageQueue_t *mq) {
    nwy_delete_msg_que((nwy_osiMessageQueue_t *)mq);
}

// Put a Message in a Queue
bool bytebeam_put_msg_que(bytebeam_osiMessageQueue_t *mq, const void *msg, uint32 timeout) {
    return nwy_put_msg_que((nwy_osiMessageQueue_t *)mq, msg, timeout);
}

// Get a Message from a Queue
bool bytebeam_get_msg_que(bytebeam_osiMessageQueue_t *mq, const void *msg, uint32 timeout) {
    return nwy_get_msg_que((nwy_osiMessageQueue_t *)mq, (void *)msg, timeout);
}

// Get the number of Pending Events in a Queue
uint32_t bytebeam_get_queue_pendingevent_cnt(bytebeam_osiMessageQueue_t *mq) {
    return nwy_get_queue_pendingevent_cnt((nwy_osiMessageQueue_t *)mq);
}

// Get the number of Space Events in a Queue
uint32_t bytebeam_get_queue_spaceevent_cnt(bytebeam_osiMessageQueue_t *mq) {
    return nwy_get_queue_spaceevent_cnt((nwy_osiMessageQueue_t *)mq);
}

//Mutex API
bytebeam_osiMutex_t *bytebeam_create_mutex() {
    return nwy_create_mutex();
}

bool bytebeam_lock_mutex(bytebeam_osiMutex_t *mutex, uint32_t timeout) {
    return nwy_lock_mutex((nwy_osiMutex_t *)mutex, timeout);
}

void bytebeam_unlock_mutex(bytebeam_osiMutex_t *mutex) {
    nwy_unlock_mutex((nwy_osiMutex_t *)mutex);
}

void bytebeam_delete_mutex(bytebeam_osiMutex_t *mutex) {
    nwy_delete_mutex((nwy_osiMutex_t *)mutex);
}

// Wrapper for Semaphore API
typedef struct 
{
    nwy_osiSemaphore_t *semaphore;
} bytebeam_semaphore_t;

bytebeam_semaphore_t *bytebeam_semaphore_create(uint32_t max_count, uint32_t init_count) 
{
    bytebeam_semaphore_t *new_semaphore = malloc(sizeof(bytebeam_semaphore_t));
    if(new_semaphore != NULL) 
    {
        new_semaphore->semaphore = nwy_semaphore_create(max_count, init_count);
    }
    return new_semaphore;
}

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

void bytebeam_semaphore_release(bytebeam_semaphore_t *sem) 
{
    if(sem != NULL) 
    {
        nwy_semahpore_release(sem->semaphore);
    }
}

void bytebeam_semaphore_delete(bytebeam_semaphore_t *sem) 
{
    if(sem != NULL) 
    {
        nwy_semahpore_delete(sem->semaphore);
        free(sem);
    }
}

void bytebeam_usleep(uint32 us)
{
    nwy_usleep(us);
}

void bytebeam_sleep(uint32 us)
{
    nwy_sleep(ms);
}
