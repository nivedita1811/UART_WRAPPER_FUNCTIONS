/**
 * @file bytebeam_osi.h
 * @brief ByteBeam Operating System Interface (OSI) functions.
 */

#ifndef BYTEBEAM_OSI_H
#define BYTEBEAM_OSI_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/** Message Queue structure. */
typedef struct bytebeam_osiMessageQueue {
    void* queue; /**< Pointer to the underlying message queue object. */
} bytebeam_osiMessageQueue_t;

/** Mutex structure. */
typedef struct bytebeam_osiMutex {
    void* mutex; /**< Pointer to the underlying mutex object. */
} bytebeam_osiMutex_t;

/**
 * @brief Creates a message queue.
 * @param msg_count The maximum number of messages that the queue can hold.
 * @param msg_size The size of each message in bytes.
 * @return A pointer to the created message queue, or NULL if creation failed.
 */
bytebeam_osiMessageQueue_t *bytebeam_create_msg_Que(uint32 msg_count, uint32 msg_size) {
    return nwy_create_msg_Que(msg_count, msg_size);
}

/**
 * @brief Deletes a message queue.
 * @param mq The message queue to delete.
 */
void bytebeam_delete_msg_que(bytebeam_osiMessageQueue_t *mq) {
    nwy_delete_msg_que((nwy_osiMessageQueue_t *)mq);
}

/**
 * @brief Puts a message in a queue.
 * @param mq The message queue.
 * @param msg The message to put in the queue.
 * @param timeout The maximum time to wait for space in the queue (in milliseconds).
 * @return True if the message was successfully put in the queue, false otherwise.
 */
bool bytebeam_put_msg_que(bytebeam_osiMessageQueue_t *mq, const void *msg, uint32 timeout) {
    return nwy_put_msg_que((nwy_osiMessageQueue_t *)mq, msg, timeout);
}

/**
 * @brief Gets a message from a queue.
 * @param mq The message queue.
 * @param msg The buffer to store the retrieved message.
 * @param timeout The maximum time to wait for a message in the queue (in milliseconds).
 * @return True if a message was successfully retrieved from the queue, false otherwise.
 */
bool bytebeam_get_msg_que(bytebeam_osiMessageQueue_t *mq, const void *msg, uint32 timeout) {
    return nwy_get_msg_que((nwy_osiMessageQueue_t *)mq, (void *)msg, timeout);
}

/**
 * @brief Gets the number of pending events in a queue.
 * @param mq The message queue.
 * @return The number of pending events in the queue.
 */
uint32_t bytebeam_get_queue_pendingevent_cnt(bytebeam_osiMessageQueue_t *mq) {
    return nwy_get_queue_pendingevent_cnt((nwy_osiMessageQueue_t *)mq);
}

/**
 * @brief Gets the number of space events in a queue.
 * @param mq The message queue.
 * @return The number of space events in the queue.
 */
uint32_t bytebeam_get_queue_spaceevent_cnt(bytebeam_osiMessageQueue_t *mq) {
    return nwy_get_queue_spaceevent_cnt((nwy_osiMessageQueue_t *)mq);
}

/**
 * @brief Creates a mutex.
 * @return A pointer to the created mutex.
 */
bytebeam_osiMutex_t *bytebeam_create_mutex() {
    return nwy_create_mutex();
}

/**
 * @brief Locks a mutex.
 * @param mutex The mutex to lock.
 * @param timeout The maximum time to wait for the mutex to become available (in milliseconds).
 * @return True if the mutex was successfully locked, false otherwise.
 */
bool bytebeam_lock_mutex(bytebeam_osiMutex_t *mutex, uint32_t timeout) {
    return nwy_lock_mutex((nwy_osiMutex_t *)mutex, timeout);
}

/**
 * @brief Unlocks a mutex.
 * @param mutex The mutex to unlock.
 */
void bytebeam_unlock_mutex(bytebeam_osiMutex_t *mutex) {
    nwy_unlock_mutex((nwy_osiMutex_t *)mutex);
}

/**
 * @brief Deletes a mutex.
 * @param mutex The mutex to delete.
 */
void bytebeam_delete_mutex(bytebeam_osiMutex_t *mutex) {
    nwy_delete_mutex((nwy_osiMutex_t *)mutex);
}

/**
 * @brief Semaphore structure.
 */
typedef struct bytebeam_semaphore {
    void* semaphore; /**< Pointer to the underlying semaphore object. */
} bytebeam_semaphore_t;

/**
 * @brief Creates a semaphore.
 * @param max_count The maximum count value of the semaphore.
 * @param init_count The initial count value of the semaphore.
 * @return A pointer to the created semaphore, or NULL if creation failed.
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
 * @brief Acquires a semaphore.
 * @param sem The semaphore to acquire.
 * @param timeout The maximum time to wait for the semaphore to become available (in milliseconds).
 * @return True if the semaphore was successfully acquired, false otherwise.
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
 * @brief Releases a semaphore.
 * @param sem The semaphore to release.
 */
void bytebeam_semaphore_release(bytebeam_semaphore_t *sem) 
{
    if(sem != NULL) 
    {
        nwy_semahpore_release(sem->semaphore);
    }
}

/**
 * @brief Deletes a semaphore.
 * @param sem The semaphore to delete.
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
 * @brief Suspends execution for a specified number of microseconds.
 * @param us The number of microseconds to sleep.
 */
void bb_usleep(uint32 us)
{
    nwy_usleep(us);
}

/**
 * @brief Suspends execution for a specified number of milliseconds.
 * @param ms The number of milliseconds to sleep.
 */
void bb_sleep(uint32 us)
{
    nwy_sleep(ms);
}

#endif /* BYTEBEAM_OSI_H */
