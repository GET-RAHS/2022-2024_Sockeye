#pragma once

#include <Arduino.h>
#include <logger.h>

#include "interfaces/Network.h"
#include "interfaces/SDCard.h"

#include "data/INA.h"
#include "data/DPS.h"
#include "data/GPS.h"
#include "data/BMS.h"
#include "data/MPPT.h"

#include <MySQL_Cursor.h> 

struct DataFrame {
    unsigned long start;
    unsigned long end;

    char query[6000] = "";
    char labels[3000] = ""; 
    char valueFormat[3000] = "";

    int n = 0;

    File* sd_file = NULL;

    DataFrame()
    {
        start = millis();
        Log(&Serial,0,"DataFrame Time start:",start);
    }
    ~DataFrame()
    {
        Log(&Serial,0,"DataFrame","exists no longer!");
    }
};

const char queryBase[] = "INSERT INTO %s.%s (%s) VALUES (%s)";

int stupid_pow(int base, int power) 
{
    int x = base;
    for (int i=0;i<power-1;i++)
    {
        x*=base;
    }
    return x;
}

int floatToIntWithPrecision(const float *x, int places)
{
    int factor = stupid_pow(10, places);
    float result = (factor*(*x));
    return (int)result;
}

void InsertV(DataFrame* dataFrame, const char* name, const char* val)
{

    strcat(dataFrame->labels, dataFrame->n != 0 ? "," : "");
    strcat(dataFrame->labels, name); 
    strcat(dataFrame->valueFormat, dataFrame->n !=0 ? "," : "");
    strcat(dataFrame->valueFormat, val); 

    Log(&Serial,0,name,val); 
    dataFrame->n++;
}

void InsertV(DataFrame* dataFrame, const char* name, const int* val)
{

    char fBuffer[12];

    sprintf(fBuffer, "%d", *val);
    strcat(dataFrame->labels, dataFrame->n != 0 ? "," : "");
    strcat(dataFrame->labels, name); 
    strcat(dataFrame->valueFormat, dataFrame->n !=0 ? "," : "");
    strcat(dataFrame->valueFormat, fBuffer); 

    Log(&Serial,0,name,*val); 
    dataFrame->n++;
}

void InsertV(DataFrame* dataFrame, const char* name, const unsigned int* val)
{

    char fBuffer[12];

    sprintf(fBuffer, "%u", *val);
    strcat(dataFrame->labels, dataFrame->n != 0 ? "," : "");
    strcat(dataFrame->labels, name); 
    strcat(dataFrame->valueFormat, dataFrame->n !=0 ? "," : "");
    strcat(dataFrame->valueFormat, fBuffer); 

    Log(&Serial,0,name,*val); 
    dataFrame->n++;
}

void InsertV(DataFrame* dataFrame, const char* name, const unsigned long* val)
{

    char fBuffer[12];

    sprintf(fBuffer, "%lu", *val);
    strcat(dataFrame->labels, dataFrame->n != 0 ? "," : "");
    strcat(dataFrame->labels, name); 
    strcat(dataFrame->valueFormat, dataFrame->n !=0 ? "," : "");
    strcat(dataFrame->valueFormat, fBuffer); 

    Log(&Serial,0,name,*val); 
    dataFrame->n++;
}

void InsertV(DataFrame* dataFrame, const char* name, const float* val, int places) 
{

    int fIntWithPrecision = floatToIntWithPrecision(val, places);

    char fBuffer[12];

    sprintf(fBuffer, "%d", fIntWithPrecision);

    strcat(dataFrame->labels, dataFrame->n != 0 ? "," : "");
    strcat(dataFrame->labels, name); 
    strcat(dataFrame->valueFormat, dataFrame->n !=0 ? "," : "");
    strcat(dataFrame->valueFormat, fBuffer); 

    Log(&Serial,0,name,*val); 
    dataFrame->n++;
}

void ConstructQuery(DataFrame* dataFrame, const char* schema, const char* table) 
{

    strcpy(dataFrame->query, "");
    sprintf(dataFrame->query, queryBase, schema, table, dataFrame->labels, dataFrame->valueFormat); 

}

void ExecuteQuery(DataFrame* dataFrame, MySQL_Cursor* cursor) 
{
    cursor->execute(dataFrame->query); 
}

