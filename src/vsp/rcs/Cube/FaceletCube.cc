/////////////////////////////////////////////////////////
// Name: FaceletCube.h
// Implements: FaceletCube
/////////////////////////////////////////////////////////

#include "vsp/rcs/Cube/FaceletCube.h"

#include <string>


#define STRING_LEN       73
#define STRING_FACE_LEN  12
#define STRING_FORMAT  "X:xxxxxxxxx,X:xxxxxxxxx,X:xxxxxxxxx,X:xxxxxxxxx,X:xxxxxxxxx,X:xxxxxxxxx,"
#define STRING_FACE    'X'
#define STRING_FACELET 'x'

const char *FaceletCube::FACE_NAMES = "FRUBLD";
const char *FaceletCube::FACE_COLORS = "wbrygo";
const char *FaceletCube::TYPE = "FaceletCube";


////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT, DESTRUCT
////////////////////////////////////////////////////////////////////////////////

FaceletCube::FaceletCube()
{
    SetInitState();
    strncpy(caMarkings, FACE_COLORS, FACE_NUMBER);
}
                          
FaceletCube::FaceletCube(FaceletCube& cube)
{
    memcpy(iaFacelets, cube.iaFacelets, FACELET_NUMBER * sizeof(int));
    strncpy(caMarkings, cube.caMarkings, FACE_NUMBER);
}
    
int FaceletCube::operator==(const FaceletCube& cube)
{
    return (memcmp(iaFacelets, cube.iaFacelets, FACELET_NUMBER * sizeof(int)) == 0);
}


////////////////////////////////////////////////////////////////////////////////
// READ/WRITE METHODS OF CUBE (FACELETS) STATE
////////////////////////////////////////////////////////////////////////////////

void FaceletCube::SetInitState()
{
    int i,j;
    for (i=0; i<FACE_NUMBER; i++)
        for(j=0; j<FACELET_IN_FACE_NUMBER; j++)
            iaFacelets[i*FACELET_IN_FACE_NUMBER+j] = (eFace) i;
}

void FaceletCube::FromTable(const int cube[]) 
     throw (CubeException)
{
     int facelet;
     eFace iaFacCopy[FaceletCube::FACELET_NUMBER];

     // Check if values in table are correct
     CheckTable(cube); 
     
     // Create backup copy of facelets
     memcpy(iaFacCopy, iaFacelets, FaceletCube::FACELET_NUMBER * sizeof(int));
     
     // Apply markings onto cube
     for (facelet=0; facelet<FaceletCube::FACELET_NUMBER; facelet++)
         iaFacelets[facelet] = (eFace) cube[facelet];

     // Try validating cube
     try
     {
         Validate();
     }
     // If validation fails, revert cube to previous state
     catch (CubeException& exp)
     {
         memcpy(iaFacelets, iaFacCopy, FaceletCube::FACELET_NUMBER * sizeof(int));
         throw;
     }
}

void FaceletCube::ToTable(int cube[]) 
{
     int facelet;
     
     // Copy numbers of markings to table
     for (facelet=0; facelet<FACELET_NUMBER; facelet++)
         cube[facelet] = iaFacelets[facelet];
}

void FaceletCube::FromString(const char* sCube) 
     throw (CubeException)
{
     // Check format, face names and facelet colors in string
     CheckFormat(sCube);
     
     // Create backup copy of facelets
     eFace iaFacCopy[FACELET_NUMBER];
     memcpy(iaFacCopy, iaFacelets, FACELET_NUMBER * sizeof(int));
     
     // Apply markings onto cube
     int face, face2, facelet;
     for (face=0; face<FACE_NUMBER; face++)
         for (face2=0; face2<FACE_NUMBER; face2++)
              if (sCube[face*STRING_FACE_LEN] == FACE_NAMES[face2])
                  for (facelet=0; facelet<FACELET_IN_FACE_NUMBER; facelet++)
                      iaFacelets[face2*FACELET_IN_FACE_NUMBER+facelet] 
                          = MarkingToFace(sCube[face*STRING_FACE_LEN+2+facelet]);
  
     // Try validating cube
     try
     {
         Validate();
     }
     // If validation fails, revert cube to previous state
     catch (CubeException& exp)
     {
         memcpy(iaFacelets, iaFacCopy, FaceletCube::FACELET_NUMBER * sizeof(int));
         throw;
     }
}

char* FaceletCube::ToString()
{
    int face, facelet;
    char *sCube = new char[STRING_LEN];

    // Init string format
    strncpy(sCube, STRING_FORMAT, STRING_LEN);

    // Fill name and markings of each face
    for (face=0; face<FaceletCube::FACE_NUMBER; face++)
    {
        sCube[face*STRING_FACE_LEN] = FaceletCube::FACE_NAMES[face];
        for (facelet=0; facelet<FaceletCube::FACELET_IN_FACE_NUMBER; facelet++)
            sCube[face*STRING_FACE_LEN+2+facelet]
                = caMarkings[iaFacelets[face*FaceletCube::FACELET_IN_FACE_NUMBER + facelet]];
    }

    return sCube;
}

void FaceletCube::Move(CubeMove::eMove move, CubeMove::eTurn turn)
     throw (CubeException)
{
    throw CubeException(this, NULL, move, 
          CubeException::ERR_FCUBE_MOVE_NOTIMPLEMENTED);
}


////////////////////////////////////////////////////////////////////////////////
// FACE / FACELET / MARKING TRANSLATION METHODS
////////////////////////////////////////////////////////////////////////////////

FaceletCube::eFace FaceletCube::FaceNameToFace(char name)
         throw (CubeException)
{
    int face;
    
    for (face=0; face<FACE_NUMBER; face++)
        if (FACE_NAMES[face] == name)
            return (eFace) face;

    throw CubeException(this, NULL, -1, 
          CubeException::ERR_FCUBE_INVALID_FACENAME);
}

FaceletCube::eFace FaceletCube::MarkingToFace(char marking) 
     throw (CubeException)
{
    int face;
    
    for (face=0; face<FACE_NUMBER; face++)
        if (caMarkings[face] == marking)
            return (eFace) face;

    throw CubeException(this, NULL, -1, 
          CubeException::ERR_FCUBE_INVALID_MARKER);    
}


////////////////////////////////////////////////////////////////////////////////
// PRIVATE CHECK AND VALIDATION METHODS
////////////////////////////////////////////////////////////////////////////////

void FaceletCube::CheckTable(const int cube[])
     throw (CubeException)
{
    int facelet;
    
    // Check if values in table are valid
    for (facelet=0; facelet<FaceletCube::FACELET_NUMBER; facelet++)
    {
        if (cube[facelet] < 0)
            throw CubeException(NULL, "", facelet, 
                CubeException::ERR_FCUBE_TABLE_VALTOOSMALL);
        if (cube[facelet] >= FaceletCube::FACE_NUMBER)
            throw CubeException(NULL, "", facelet, 
                CubeException::ERR_FCUBE_TABLE_VALTOOBIG);
    }

}

void FaceletCube::CheckFormat(const char* sCube)
     throw (CubeException)
{
    int i;
    int face, faceUsed[FaceletCube::FACE_NUMBER];
     
    // Check if sCube exists
    if (sCube==NULL)
        throw CubeException(NULL, NULL, 0, 
            CubeException::ERR_FCUBE_PARSE_STRINGNULL);
    
    // Check length of string
    if (strlen(sCube) < STRING_LEN-1)
        throw CubeException(NULL, sCube, strlen(sCube), 
            CubeException::ERR_FCUBE_PARSE_STRINGTOOSHORT);
    
    // Set markings of faces
    for (i=0; i<FaceletCube::FACE_NUMBER; i++)
        for (face=0; face<FACE_NUMBER; face++)
            if (sCube[i*STRING_FACE_LEN] == FACE_NAMES[face])
                caMarkings[face] = sCube[i*STRING_FACE_LEN+2+4];
    
    // Set all faced unused
    for (i=0; i<FaceletCube::FACE_NUMBER; i++)
        faceUsed[i] = 0;

    // Check format and markings of the string
    for (i=0; i<STRING_LEN; i++)
    {
        // Check if face name is valid
        if (STRING_FORMAT[i] == STRING_FACE)
        {
            if (strchr(FACE_NAMES, sCube[i]) == NULL)
                throw CubeException(NULL, sCube, i, 
                    CubeException::ERR_FCUBE_PARSE_FACEINVALID);
            faceUsed[FaceNameToFace(sCube[i])]++;
        }
        // Check if facelet color is valid
        else if (STRING_FORMAT[i] == STRING_FACELET)
        {
            if (strchr(caMarkings, sCube[i]) == NULL)
                throw CubeException(NULL, sCube, i, 
                    CubeException::ERR_FCUBE_PARSE_FACELETINVALID);
        }
        // Check if format is valid
        else
        {
            if (sCube[i] != STRING_FORMAT[i])
                throw CubeException(NULL, sCube, i, 
                    CubeException::ERR_FCUBE_PARSE_FORMATINVALID);
        }
    }
    
    // Check if all faces defined exactly once
    for (face=0; face<FaceletCube::FACE_NUMBER; face++)
        if (faceUsed[face] != 1)
            throw CubeException(NULL, sCube, face, 
                CubeException::ERR_FCUBE_PARSE_FACENAMEREPEATED);
}

void FaceletCube::Validate() 
     throw (CubeException)
{
    ValidateCenters();
    ValidateFacelets();
}

void FaceletCube::ValidateCenters() 
     throw (CubeException)
{
    int face1,face2;

    // Validate only in validation mode
    if (!bValidation)
        return;
        
    // Check if centers are unique
    for (face1=0; face1<FACE_NUMBER; face1++)
        for (face2=0; face2<face1; face2++)
            if (iaFacelets[face1*FACELET_IN_FACE_NUMBER + 4] 
                    == iaFacelets[face2*FACELET_IN_FACE_NUMBER + 4])
                throw CubeException(this, NULL, face1, 
                      CubeException::ERR_FCUBE_DUPLICATE_CENTER_MARKING);
}

void FaceletCube::ValidateFacelets() 
     throw (CubeException)
{
    int face, facelet;
    int counter[6];

    // Validate only in validation mode
    if (!bValidation)
        return;
        
    // Clear counter of number of occurance of facelet marking
    for (face = 0; face < FaceletCube::FACE_NUMBER; face++)
        counter[face] = 0;

    // Count number of occurance facelet markings
    for (facelet = 0; facelet < FaceletCube::FACELET_NUMBER; facelet++) 
    {
        // Check if marking of facelet is of one of markings of faces
        if (iaFacelets[facelet] >= FACE_NUMBER || iaFacelets[facelet] < 0)
            throw CubeException(this, NULL, facelet, 
                  CubeException::ERR_FCUBE_INVALID_MARKER);
        
        counter[iaFacelets[facelet]]++;
    }

    // Check if each marking occurs eactly 9 times
    for (face = 0; face < FaceletCube::FACE_NUMBER; face++) 
        if (counter[face] != FaceletCube::FACELET_IN_FACE_NUMBER)
            throw CubeException(this, NULL, face, 
                  CubeException::ERR_FCUBE_INVALID_FACELETCOUNT);
}
