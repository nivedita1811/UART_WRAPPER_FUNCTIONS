#include "bb_file.h"

bool bytebeam_sdcard_mount()
{
    return nwy_sdk_sdcard_mount();
}

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

void bytebeam_sdcard_format()
{
    nwy_format_sdcard();
}

void bytebeam_sdcard_unmount()
{
    nwy_sdk_sdcard_unmount();
}

unsigned long long bytebeam_sdcard_available_size()
{
    return nwy_sdk_vfs_free_size("/sdcard0/");
}

int bytebeam_dir_create(const char* name)
{
    return nwy_sdk_vfs_mkdir(name);
}

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

int bytebeam_dir_destroy(const char* name)
{
    return nwy_sdk_vfs_rmdir_recursive(name);
}

bytebeam_fdes bytebeam_fopen(const char* path, bytebeam_fmode_e mode)
{
    return nwy_sdk_fopen(path, mode);
}

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

int bytebeam_fdelete(const char* path)
{
    return nwy_sdk_file_unlink(path);
}

bool bytebeam_fexist(const char* path)
{
    return nwy_sdk_fexist(path);
}

int bytebeam_frename(const char* old_path, const char* new_path)
{
    return nwy_sdk_frename(old_path, new_path);
}

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

int bytebeam_fclose(bytebeam_fdes file)
{
    return nwy_sdk_fclose(file);
}