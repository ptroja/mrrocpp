/////////////////////////////////////////////////////////
// Name: DataTable.cpp
// Implements : DataTable
/////////////////////////////////////////////////////////

#include "vsp/rcs/General/DataTable.h"


bool DataTable::bLog = false;
char* DataTable::sFolderName = NULL;


////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT, DESTRUCT
////////////////////////////////////////////////////////////////////////////////

DataTable::DataTable(int size) : iSize(size), iaTable(NULL)
{ 
    //if (size==0) throw Exception;
    iaTable = new int[iSize];
}

DataTable::DataTable(DataTable& table) : iSize(table.iSize), iaTable(NULL)
{ 
    //if (size==0) throw Exception;
    iaTable = new int[iSize];
    memcpy(iaTable, table.iaTable, iSize*sizeof(int));
}

DataTable::~DataTable()
{
    delete[] iaTable;
}

DataTable& DataTable::operator=(DataTable& table)
{
    if (this!=&table)
    {
        delete[] iaTable;
        iSize = table.iSize;
        iaTable = new int[iSize];
        memcpy(iaTable, table.iaTable, iSize*sizeof(int));
    }
    return *this;
}
    

////////////////////////////////////////////////////////////////////////////////
// CORE METHODS - INITIATE (public) 
//                and (private) SAVING TO AND LOADING FROM FILE
////////////////////////////////////////////////////////////////////////////////

void DataTable::Init() 
{
    // get full path of the file
    const char* folder = GetFolderName();
    const char* file = GetName();
    const char* ext = GetExtention();
    int follen = strlen(folder);
    int fillen = strlen(file);
    int extlen = strlen(ext);
    char *path = new char[follen + fillen + extlen + 2];
    strncpy(path, folder, follen);
    strncpy(&path[follen], file, fillen);
    path[follen+fillen] = '.';
    strncpy(&path[follen+fillen+1], ext, extlen);
    path[follen+fillen+1+extlen] = '\0';
    
    // try opening a file with table data
    FILE *file_istream;
    file_istream = fopen(path, "rb");

    // if opening failed, generate tables
    if (file_istream == NULL) 
    {
        if (bLog) printf("Move table create %s %s\n", path, GetDescription());
        Generate();

        if (bLog) printf("Move table save %s", path);
        FILE *file_ostream;
        file_ostream = fopen(path, "wb");
        Save(file_ostream);
        fclose(file_ostream);
        if (bLog) printf(" - OK.\n");
    }    
    
    // if file is successfully opened, upload table data
    else 
    {
        if (bLog) printf("Move table read %s %s\n", path, GetDescription());
        Load(file_istream);
        fclose(file_istream);
    }    
    
    delete[] path;
}
  
void DataTable::Save(FILE *outputStream)
{
    int size = iSize * sizeof(int) / sizeof(char);
    fwrite((char*)iaTable, sizeof(char), size, outputStream);
}
    
void DataTable::Load(FILE *inputStream)
{
    int size = iSize * sizeof(int) / sizeof(char);
    fread((char*)iaTable, sizeof(char), size, inputStream);
}
