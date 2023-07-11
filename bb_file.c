#include "bb_file.h"

/** 
================================================================================================================================================

@fn Name             : bool bytebeam_sdcard_mount()
@b Scope             : Public
@n@n@b Description   : This function mounts the SD card. 
@return Return Value : Returns true if the mounting is successful, false otherwise.

================================================================================================================================================
*/
bool bytebeam_sdcard_mount()
{
    return nwy_sdk_sdcard_mount();
}

/** 
================================================================================================================================================

@fn Name             : bool bytebeam_sdcard_status()
@b Scope             : Public
@n@n@b Description   : This function checks the status of the SD card. 
@return Return Value : Returns true if the SD card is ready, false otherwise.

================================================================================================================================================
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
================================================================================================================================================

@fn Name             : void bytebeam_sdcard_format()
@b Scope             : Public
@n@n@b Description   : This function formats the SD card.
@return Return Value : None.

================================================================================================================================================
*/
void bytebeam_sdcard_format()
{
    nwy_format_sdcard();
}

/** 
================================================================================================================================================

@fn Name             : void bytebeam_sdcard_unmount()
@b Scope             : Public
@n@n@b Description   : This function unmounts the SD card.
@return Return Value : None.

================================================================================================================================================
*/

void bytebeam_sdcard_unmount()
{
    nwy_sdk_sdcard_unmount();
}

/** 
================================================================================================================================================

@fn Name             : unsigned long long bytebeam_sdcard_available_size()
@b Scope             : Public
@n@n@b Description   : This function gets the available size of the SD card. 
@return Return Value : Returns the available size of the SD card.

================================================================================================================================================
*/
unsigned long long bytebeam_sdcard_available_size()
{
    return nwy_sdk_vfs_free_size("/sdcard0/");
}

/** 
================================================================================================================================================

@fn Name             : int bytebeam_dir_create(const char* name)
@b Scope             : Public
@n@n@b Description   : This function creates a directory with the given name. 
@param Input Data    : Name of the directory.
@return Return Value : Returns status code.

================================================================================================================================================
*/
int bytebeam_dir_create(const char* name)
{
    return nwy_sdk_vfs_mkdir(name);
}

/** 
================================================================================================================================================

@fn Name             : bool bytebeam_dir_exist(const char* name)
@b Scope             : Public
@n@n@b Description   : This function checks if a directory with the given name exists. 
@param Input Data    : Name of the directory.
@return Return Value : Returns true if the directory exists, false otherwise.

================================================================================================================================================
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
================================================================================================================================================

@fn Name             : int bytebeam_dir_destroy(const char* name)
@b Scope             : Public
@n@n@b Description   : This function destroys a directory with the given name. 
@param Input Data    : Name of the directory.
@return Return Value : Returns status code.

================================================================================================================================================
*/
int bytebeam_dir_destroy(const char* name)
{
    return nwy_sdk_vfs_rmdir_recursive(name);
}

/** 
================================================================================================================================================

@fn Name             : bytebeam_fdes bytebeam_fopen(const char* path, bytebeam_fmode_e mode)
@b Scope             : Public
@n@n@b Description   : This function opens a file with the given mode. 
@param Input Data    : Path of the file and the file mode.
@return Return Value : Returns a file descriptor.

================================================================================================================================================
*/
bytebeam_fdes bytebeam_fopen(const char* path, bytebeam_fmode_e mode)
{
    return nwy_sdk_fopen(path, mode);
}

/** 
================================================================================================================================================

@fn Name             : int bytebeam_fread(bytebeam_fdes file, void* dst, size_t len)
@b Scope             : Public
@n@n@b Description   : This function reads from a file into a destination buffer. 
@param Input Data    : File descriptor, destination buffer, and length of bytes to read.
@return Return Value : Returns status code.

================================================================================================================================================
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
================================================================================================================================================

@fn Name             : int bytebeam_fwrite(bytebeam_fdes file, void* src, size_t len)
@b Scope             : Public
@n@n@b Description   : This function writes to a file from a source buffer. 
@param Input Data    : File descriptor, source buffer, and length of bytes to write.
@return Return Value : Returns status code.

================================================================================================================================================
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
================================================================================================================================================

@fn Name             : long bytebeam_fsize(const char* path)
@b Scope             : Public
@n@n@b Description   : This function gets the size of a file. 
@param Input Data    : Path of the file.
@return Return Value : Returns the size of the file.

================================================================================================================================================
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
================================================================================================================================================

@fn Name             : int bytebeam_fdelete(const char* path)
@b Scope             : Public
@n@n@b Description   : This function deletes a file. 
@param Input Data    : Path of the file.
@return Return Value : Returns status code.

================================================================================================================================================
*/
int bytebeam_fdelete(const char* path)
{
    return nwy_sdk_file_unlink(path);
}

/** 
================================================================================================================================================

@fn Name             : bool bytebeam_fexist(const char* path)
@b Scope             : Public
@n@n@b Description   : This function checks if a file exists. 
@param Input Data    : Path of the file.
@return Return Value : Returns true if the file exists, false otherwise.

================================================================================================================================================
*/
bool bytebeam_fexist(const char* path)
{
    return nwy_sdk_fexist(path);
}

/** 
================================================================================================================================================

@fn Name             : int bytebeam_frename(const char* old_path, const char* new_path)
@b Scope             : Public
@n@n@b Description   : This function renames a file. 
@param Input Data    : Old and new path of the file.
@return Return Value : Returns status code.

================================================================================================================================================
*/
int bytebeam_frename(const char* old_path, const char* new_path)
{
    return nwy_sdk_frename(old_path, new_path);
}

/** 
================================================================================================================================================

@fn Name             : int bytebeam_fseek(bytebeam_fdes file, int offset, bytebeam_fseek_offset_e whence)
@b Scope             : Public
@n@n@b Description   : This function sets the file pointer to a new location. 
@param Input Data    : File descriptor, offset and whence indicator.
@return Return Value : Returns status code.

================================================================================================================================================
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
================================================================================================================================================

@fn Name             : int bytebeam_fclose(bytebeam_fdes file)
@b Scope             : Public
@n@n@b Description   : This function closes a file. 
@param Input Data    : File descriptor.
@return Return Value : Returns status code.

================================================================================================================================================
*/
int bytebeam_fclose(bytebeam_fdes file)
{
    return nwy_sdk_fclose(file);
}
