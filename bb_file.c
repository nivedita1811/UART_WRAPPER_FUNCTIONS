/**
 * @file bb_file.h
 * @brief ByteBeam File System API functions.
 */

#ifndef BB_FILE_H
#define BB_FILE_H

#include "bb_file.h"

/**
 * @brief Mounts the SD card.
 * @return True if successful, false otherwise.
 */
bool bytebeam_sdcard_mount()
{
    return nwy_sdk_sdcard_mount();
}

/**
 * @brief Checks the status of the SD card.
 * @return True if the SD card is available, false otherwise.
 */
bool bytebeam_sdcard_status()
{
    int status = 0;

    status = nwy_read_sdcart_status();

    if(status != 1)
    {
        return false;
    }

    return true;
}

/**
 * @brief Formats the SD card.
 */
void bytebeam_sdcard_format()
{
    nwy_format_sdcard();
}

/**
 * @brief Unmounts the SD card.
 */
void bytebeam_sdcard_unmount()
{
    nwy_sdk_sdcard_unmount();
}

/**
 * @brief Retrieves the available size of the SD card.
 * @return The available size in bytes.
 */
unsigned long long bytebeam_sdcard_available_size()
{
    return nwy_sdk_vfs_free_size("/sdcard0/");
}

/**
 * @brief Creates a directory.
 * @param name The name of the directory to create.
 * @return 0 if successful, -1 otherwise.
 */
int bytebeam_dir_create(const char* name)
{
    return nwy_sdk_vfs_mkdir(name);
}


/**
 * @brief Checks if a directory exists.
 * @param name The name of the directory to check.
 * @return True if the directory exists, false otherwise.
 */
bool bytebeam_dir_exist(const char* name)
{
    int ret_code = 0;
    nwy_dir* p_dir = NULL;

    p_dir = nwy_sdk_vfs_opendir(name);

    if(name == NULL)
    {
        return false;
    }

    ret_code = nwy_sdk_vfs_closedir(p_dir);

    if(ret_code != 0)
    {
        return false;
    }
    
    return true;
}

/**
 * @brief Destroys a directory.
 * @param name The name of the directory to destroy.
 * @return 0 if successful, -1 otherwise.
 */
int bytebeam_dir_destroy(const char* name)
{
    return nwy_sdk_vfs_rmdir_recursive(name);
}

/**
 * @brief Opens a file.
 * @param path The path to the file.
 * @param mode The file mode.
 * @return The file descriptor if successful, -1 otherwise.
 */
bytebeam_fdes bytebeam_fopen(const char* path, bytebeam_fmode_e mode)
{
    return nwy_sdk_fopen(path, mode);
}

/**
 * @brief Reads data from a file.
 * @param file The file descriptor.
 * @param dst The destination buffer to store the read data.
 * @param len The number of bytes to read.
 * @return The number of bytes read if successful, a negative value otherwise.
 */
int bytebeam_fread(bytebeam_fdes file, void* dst, size_t len)
{
    size_t bytes_read = 0;

    bytes_read = nwy_sdk_fread(file, dst, len);

    if(bytes_read != len)
    {
        return bytes_read - len;
    }

    return BYTEBEAM_FSUCCESS;
}

/**
 * @brief Writes data to a file.
 * @param file The file descriptor.
 * @param src The source buffer containing the data to write.
 * @param len The number of bytes to write.
 * @return The number of bytes written if successful, a negative value otherwise.
 */
int bytebeam_fwrite(bytebeam_fdes file, void* src, size_t len)
{
    size_t bytes_written = 0;

    bytes_written = nwy_sdk_fwrite(file, src, len);

    if(bytes_written != len)
    {
        return bytes_written - len;
    }

    return BYTEBEAM_FSUCCESS;
}

/**
 * @brief Retrieves the size of a file.
 * @param path The path to the file.
 * @return The size of the file in bytes, or 0 if an error occurred.
 */
long bytebeam_fsize(const char* path)
{
    long size = 0;
    
    size = nwy_sdk_fsize(path);

    if(size < 0)
    {
        return 0;
    }

    return size;
}

/**
 * @brief Deletes a file.
 * @param path The path to the file.
 * @return 0 if successful, -1 otherwise.
 */
int bytebeam_fdelete(const char* path)
{
    return nwy_sdk_file_unlink(path);
}

/**
 * @brief Checks if a file exists.
 * @param path The path to the file.
 * @return True if the file exists, false otherwise.
 */
bool bytebeam_fexist(const char* path)
{
    return nwy_sdk_fexist(path);
}

/**
 * @brief Renames a file or directory.
 * @param old_path The current path of the file or directory.
 * @param new_path The new path of the file or directory.
 * @return 0 if successful, -1 otherwise.
 */
int bytebeam_frename(const char* old_path, const char* new_path)
{
    return nwy_sdk_frename(old_path, new_path);
}

/**
 * @brief Sets the file position indicator of a file.
 * @param file The file descriptor.
 * @param offset The offset from the reference point specified by whence.
 * @param whence The reference point for the offset.
 * @return 0 if successful, -1 otherwise.
 */
int bytebeam_fseek(bytebeam_fdes file, int offset, bytebeam_fseek_offset_e whence)
{
    int ret_offset = 0;

    ret_offset = nwy_sdk_fseek(file, offset, whence);

    if(ret_offset != offset)
    {
        return ret_offset - offset;
    }

    return BYTEBEAM_FSUCCESS;
}

/**
 * @brief Closes a file.
 * @param file The file descriptor.
 * @return 0 if successful, -1 otherwise.
 */
int bytebeam_fclose(bytebeam_fdes file)
{
    return nwy_sdk_fclose(file);
}

#endif /* BB_FILE_H */
