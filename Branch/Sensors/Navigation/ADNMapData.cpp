/*
*   Autonomous Driving Navigation Library
*   Map Road Line Data Class
*   Li Bing, QYJY, 2018.7
*/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "ADNMapData.h"

ADNMapData::ADNMapData()
{
    m_LineInfos=NULL;
    m_ShapeData=NULL;
}

ADNMapData::ADNMapData(const char *path)
{
    m_LineInfos=NULL;
    m_ShapeData=NULL;
    Load(path);
}

ADNMapData::~ADNMapData()
{
    if(m_LineInfos!=NULL)free(m_LineInfos);
    if(m_ShapeData!=NULL)free(m_ShapeData);
}

bool ADNMapData::Load(const char *path)
{
    FILE* file=fopen(path,"rb");
    if(file==NULL)
    {
        return false;
    }
    if(fread(&m_Head,sizeof(Head),1,file)!=1)
    {
        fclose(file);
        return false;
    }
    if(m_Head.Count==0)
    {
        fclose(file);
        return true;
    }
    m_LineInfos=(Info*)calloc(m_Head.Count,sizeof(Info));
    if(m_LineInfos==NULL)
    {
        fclose(file);
        return false;
    }
    if(fread(m_LineInfos,sizeof(Info),m_Head.Count,file)!=m_Head.Count)
    {
        fclose(file);
        return false;
    }
    int count_points=0;
    for(int i=0;i<m_Head.Count;i++)
    {
        count_points+=m_LineInfos[i].Count;
    }
    m_ShapeData=(ADNPoint*)calloc(count_points,sizeof(ADNPoint));
    if(m_LineInfos==NULL)
    {
        fclose(file);
        return false;
    }
    if(fread(m_ShapeData,sizeof(ADNPoint),count_points,file)!=count_points)
    {
        fclose(file);
        return false;
    }
    fclose(file);
    return true;
}

ADNInt ADNMapData::LineCount()
{
    return m_Head.Count;
}

ADNRoadLine ADNMapData::GetLine(ADNInt idx)
{
    assert(idx>=0&&idx<m_Head.Count);
    ADNRoadLine line;
    line.Id=m_LineInfos[idx].Id;
    line.Direction=m_LineInfos[idx].Direction;
    line.BBox=m_LineInfos[idx].BBox;
    line.Curve.Count=m_LineInfos[idx].Count;
    line.Curve.Points=m_ShapeData+m_LineInfos[idx].Index;
    return line;
}
