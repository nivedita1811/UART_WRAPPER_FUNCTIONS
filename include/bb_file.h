#ifndef BYTEBEAM_FILE_H
#define BYTEBEAM_FILE_H

#include <stdbool.h>

#ifndef BYTEBEAM_FSUCCESS
#define BYTEBEAM_FSUCCESS 0
#endif

typedef int bytebeam_fdes;

/**
=====================================================================================================================================================

@enum Name           : bytebeam_fmode
@b Scope             : Public
@n@n@b Description   : Enumerates the modes in which a file can be opened.
@param Input Data    : None
@return Return Value : None

=====================================================================================================================================================
*/
typedef enum bytebeam_fmode {
    BYTEBEAM_RDONLY   = 0x0,   // Read-only mode
    BYTEBEAM_WRONLY   = 0x1,   // Write-only mode
    BYTEBEAM_RDWR     = 0x2,   // Read and write mode
    BYTEBEAM_APPEND   = 0x8,   // Append mode
    BYTEBEAM_CREAT    = 0x0200, // Create mode
    BYTEBEAM_TRUNC    = 0x0400 // Truncate mode
} bytebeam_fmode_e;

/**
=====================================================================================================================================================

@enum Name           : bytebeam_fseek_offset
@b Scope             : Public
@n@n@b Description   : Enumerates the possible file seek positions.
@param Input Data    : None
@return Return Value : None

=====================================================================================================================================================
*/
typedef enum bytebeam_fseek_offset {
    BYTEBEAM_SEEK_SET       = 0, // Seek from the beginning of the file
    BYTEBEAM_NWY_SEEK_CUR   = 1, // Seek from the current position
    BYTEBEAM_NWY_SEEK_END   = 2  // Seek from the end of the file
} bytebeam_fseek_offset_e;

// sd card functions
bool bytebeam_sdcard_mount();
bool bytebeam_sdcard_status();
void bytebeam_sdcard_format();
void bytebeam_sdcard_unmount();
unsigned long long bytebeam_sdcard_available_size();

// directory functions
int bytebeam_dir_create(const char* name);
bool bytebeam_dir_exist(const char* name);
int bytebeam_dir_destroy(const char* name);

// file system functions
bytebeam_fdes bytebeam_fopen(const char* path, bytebeam_fmode_e mode);
int bytebeam_fread(bytebeam_fdes file, void* dst, size_t len);
int bytebeam_fwrite(bytebeam_fdes file, void* src, size_t len);
long bytebeam_fsize(const char* path);
int bytebeam_fdelete(const char* path);
bool bytebeam_fexist(const char* path);
int bytebeam_frename(const char* old_path, const char* new_path);
int bytebeam_fseek(bytebeam_fdes file, int offset, bytebeam_fseek_offset_e whence);
int bytebeam_fclose(bytebeam_fdes file);

#endif /* BYTEBEAM_FILE_H */
