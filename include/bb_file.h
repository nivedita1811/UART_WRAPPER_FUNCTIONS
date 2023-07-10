#ifndef BYTEBEAM_FILE_H
#define BYTEBEAM_FILE_H

#include <stdbool.h>

/**
 * @file bytebeam_file.h
 * @brief File I/O operations for ByteBeam.
 */

#ifndef BYTEBEAM_FSUCCESS
#define BYTEBEAM_FSUCCESS 0   /**< File operation success return value. */
#endif

typedef int bytebeam_fdes; /**< File descriptor type. */

typedef enum bytebeam_fmode {
    BYTEBEAM_RDONLY   = 0x0, /**< Read-only mode. */
    BYTEBEAM_WRONLY   = 0x1, /**< Write-only mode. */
    BYTEBEAM_RDWR     = 0x2, /**< Read-write mode. */
    BYTEBEAM_APPEND   = 0x8, /**< Append mode. */
    BYTEBEAM_CREAT    = 0x0200, /**< Create file if it does not exist. */
    BYTEBEAM_TRUNC    = 0x0400, /**< Truncate file if it exists. */
} bytebeam_fmode_e;

typedef enum bytebeam_fseek_offset {
    BYTEBEAM_SEEK_SET       = 0, /**< Seek from the beginning of the file. */
    BYTEBEAM_NWY_SEEK_CUR   = 1, /**< Seek from the current position. */
    BYTEBEAM_NWY_SEEK_END   = 2, /**< Seek from the end of the file. */
} bytebeam_fseek_offset_e;

/**
 * @brief Mounts the SD card.
 * @return True if the SD card was successfully mounted, false otherwise.
 */

bool bytebeam_sdcard_mount();

/**
 * @brief Checks the status of the SD card.
 * @return True if the SD card is available and mounted, false otherwise.
 */

bool bytebeam_sdcard_status();

/**
 * @brief Formats the SD card.
 */

void bytebeam_sdcard_format();

/**
 * @brief Unmounts the SD card.
 */

void bytebeam_sdcard_unmount();

/**
 * @brief Gets the available size of the SD card.
 * @return The available size of the SD card in bytes.
 */

unsigned long long bytebeam_sdcard_available_size();

/**
 * @brief Creates a directory.
 * @param name The name of the directory.
 * @return 0 if the directory was successfully created, or an error code if an error occurred.
 */

int bytebeam_dir_create(const char* name);

/**
 * @brief Checks if a directory exists.
 * @param name The name of the directory.
 * @return True if the directory exists, false otherwise.
 */

bool bytebeam_dir_exist(const char* name);

/**
 * @brief Destroys a directory.
 * @param name The name of the directory.
 * @return 0 if the directory was successfully destroyed, or an error code if an error occurred.
 */

int bytebeam_dir_destroy(const char* name);

/**
 * @brief Opens a file.
 * @param path The path to the file.
 * @param mode The file open mode.
 * @return The file descriptor if the file was successfully opened, or an error code if an error occurred.
 */

bytebeam_fdes bytebeam_fopen(const char* path, bytebeam_fmode_e mode);

/**
 * @brief Reads data from a file.
 * @param file The file descriptor.
 * @param dst The destination buffer.
 * @param len The number of bytes to read.
 * @return The number of bytes read, or an error code if an error occurred.
 */

int bytebeam_fread(bytebeam_fdes file, void* dst, size_t len);

/**
 * @brief Writes data to a file.
 * @param file The file descriptor.
 * @param src The source buffer.
 * @param len The number of bytes to write.
 * @return The number of bytes written, or an error code if an error occurred.
 */

int bytebeam_fwrite(bytebeam_fdes file, void* src, size_t len);

/**
 * @brief Gets the size of a file.
 * @param path The path to the file.
 * @return The size of the file in bytes, or an error code if an error occurred.
 */

long bytebeam_fsize(const char* path);

/**
 * @brief Deletes a file.
 * @param path The path to the file.
 * @return 0 if the file was successfully deleted, or an error code if an error occurred.
 */

int bytebeam_fdelete(const char* path);

/**
 * @brief Checks if a file exists.
 * @param path The path to the file.
 * @return True if the file exists, false otherwise.
 */

bool bytebeam_fexist(const char* path);

/**
 * @brief Renames a file.
 * @param old_path The path to the old file.
 * @param new_path The path to the new file.
 * @return 0 if the file was successfully renamed, or an error code if an error occurred.
 */

int bytebeam_frename(const char* old_path, const char* new_path);

/**
 * @brief Sets the file position indicator.
 * @param file The file descriptor.
 * @param offset The offset value.
 * @param whence The position from which the offset is calculated.
 * @return 0 if the file position was successfully set, or an error code if an error occurred.
 */

int bytebeam_fseek(bytebeam_fdes file, int offset, bytebeam_fseek_offset_e whence);

/**
 * @brief Closes a file.
 * @param file The file descriptor.
 * @return 0 if the file was successfully closed, or an error code if an error occurred.
 */

int bytebeam_fclose(bytebeam_fdes file);

#endif /* BYTEBEAM_FILE_H */
