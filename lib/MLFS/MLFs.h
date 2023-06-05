#ifndef _ML_FS_H_
#define _ML_FS_H_

#include "SPIFFS.h"
#include "FS.h"
#define DEBUG 1
#ifdef DEBUG
#define DB_PRINTLN(x) Serial.println(x)
#define DB_PRINT(x) Serial.print(x)
#else
#define DB_PRINTLN(x)
#define DB_PRINT(x)
#endif
class MLFs
{
private:
    /* data */
public:
    MLFs(/* args */);
    ~MLFs();
    void init();
    void writeFile(fs::FS &fs, const char *path, const char *message);
    String readFile(fs::FS &fs, const char *path);
    void createDir(fs::FS &fs, const char *path);
    void removeDir(fs::FS &fs, const char * path);
    void appendFile(fs::FS &fs, const char * path, const char * message);
    void renameFile(fs::FS &fs, const char * path1, const char * path2);
    void deleteFile(fs::FS &fs, const char * path);
};

#endif