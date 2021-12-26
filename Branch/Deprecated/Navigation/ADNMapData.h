/*
*   Autonomous Driving Navigation Library
*   Map Road Line Data Class
*   Li Bing, QYJY, 2018.7
*/

#ifndef ADNMAPDATA_H
#define ADNMAPDATA_H

#include "ADNType.h"

// Map Road Line Data Class
class ADNMapData
{
public:
    ADNMapData();
    // construct with data path
    ADNMapData(const char *path);
    ~ADNMapData();

public:
    // load data file
    bool Load(const char *path);

    // get road line count
    ADNInt LineCount();

    // get road line
    ADNRoadLine GetLine(ADNInt idx);

private:
    // data head
    struct Head
    {
        ADNInt Type;    //shape type
        ADNInt Count;   //shape count
        ADNInt Reserved1;
        ADNInt Reserved2;
    };
    // shape info
    struct Info
    {
        ADNInt Id;      //shape id
        ADNInt Direction;   //direction
        ADNInt Count;       //shape points count
        ADNInt Index;       //shape data start index
        ADNRect BBox;       //bound box
    };

private:
    Head m_Head;    //data head info
    Info* m_LineInfos;  //road line info data
    ADNPoint *m_ShapeData;  //road line shape points data
};

#endif // ADNMAPDATA_H
