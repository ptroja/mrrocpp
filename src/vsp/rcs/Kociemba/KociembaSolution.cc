/////////////////////////////////////////////////////////
// Name: KociembaSolution.cpp
// Implements: KociembaSolution
/////////////////////////////////////////////////////////


#include "vsp/rcs/Kociemba/KociembaSolution.h"

////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT
////////////////////////////////////////////////////////////////////////////////

KociembaSolution::KociembaSolution() 
    : CubeSolution(), iPhase1Length(0)
{
}

KociembaSolution::KociembaSolution(const KociembaSolution& sol)
    : CubeSolution((CubeSolution) sol), iPhase1Length(sol.iPhase1Length)
{
}

KociembaSolution::KociembaSolution(const CubeSolution& sol1, const CubeSolution& sol2)
    : CubeSolution(), iPhase1Length(sol1.GetLength())
{
    int len1 = sol1.GetLength();
    int len2 = sol2.GetLength();
    
    if (sol1.GetInfo(INFO_ALREADY_SOLVED) && sol2.GetInfo(INFO_ALREADY_SOLVED))
        SetInfo(INFO_ALREADY_SOLVED, true);
    else if (sol1.GetInfo(INFO_ALREADY_SOLVED) || sol2.GetInfo(INFO_ALREADY_SOLVED))
        SetInfo(INFO_OPTIMAL, true);
    
    int i;
    CubeMove::eMove move;
    CubeMove::eTurn turn;
    for (i=0; i<len1; i++)
    {
        sol1.GetMove(i, move, turn);
        SetMove(i, move, turn);
    }
    for (i=0; i<len2; i++)
    {
        sol2.GetMove(i, move, turn);
        SetMove(i + len1, move, turn);
    }
}

KociembaSolution& KociembaSolution::operator=(const KociembaSolution& sol)
{
    if (this!=&sol)
    {
        CubeSolution::operator=(*((CubeSolution*) &sol));
        iPhase1Length = sol.iPhase1Length;
    }
    return *this;
}


////////////////////////////////////////////////////////////////////////////////
// TOSTRING METHOD
////////////////////////////////////////////////////////////////////////////////

void KociembaSolution::FromString(const char* sSol) 
     throw (Exception)
{
    // Check format, move names and turns in string
    CheckFormat(sSol);
     
    CubeSolution p1sol, p2sol;
    int start, len;
    char *sol;
    
    // get phase1 solution
    start = 1;
    len = strchr(sSol+start, ')') - (sSol+start);
    if (len > 0)
    {
        sol = new char[len+1];
        strncpy(sol, sSol+start, len);
        sol[len] = '\0';
        try 
        {
            p1sol.FromString(sol);
        }
        catch (CubeSolutionException &exp)
        {
            delete[] sol;
            throw KociembaException(KociembaException::ERR_SOLUTION_PARSE_P1MOVEINVALID
                + exp.GetError() - CubeSolutionException::ERR_CSOL_PARSE_MOVEINVALID);
        }
        delete[] sol;
    }
    
    // get phase2 solution
    start += len+3;
    len = strchr(sSol+start, ')') - (sSol+start);
    if (len > 0)
    {
        sol = new char[len+1];
        strncpy(sol, sSol+start, len);
        sol[len] = '\0';
        try 
        {
            p2sol.FromString(sol);
        }
        catch (CubeSolutionException &exp)
        {
            delete[] sol;
            throw KociembaException(KociembaException::ERR_SOLUTION_PARSE_P2MOVEINVALID
                + exp.GetError() - CubeSolutionException::ERR_CSOL_PARSE_MOVEINVALID);
        }
        delete[] sol;
    }
            
    // create KociembaSolution and copy it to this
    KociembaSolution ksol(p1sol, p2sol);
    operator=(ksol);
}

char* KociembaSolution::ToString() const
{
    char* str = CubeSolution::ToString();
    char* sSol = new char[strlen(str)+6];
    int p2len = GetLength() - iPhase1Length;
    
    int pos = 0;
    
    // phase1 solution
    if (iPhase1Length)
    {
        sSol[0] = '(';
        strncpy(&sSol[1], str, 3*iPhase1Length-1);
        sSol[3*iPhase1Length] = ')';
        sSol[3*iPhase1Length+1] = ',';
        pos = 3*iPhase1Length+2;
    }
    else
    {
        strncpy(sSol, "(),", 3);
        pos = 3;
    }
    
    // phase2 solution
    if (p2len)
    {
        sSol[pos] = '(';
        strncpy(&sSol[pos+1], &str[3*iPhase1Length], 3*p2len-1);
        sSol[pos+3*p2len] = ')';
        pos += 3*p2len+1;
    }
    else
    {
        strncpy(&sSol[pos], "()", 3);
        pos += 2;
    }
    sSol[pos] = '\0';
    
    delete[] str;
    return sSol;
}


////////////////////////////////////////////////////////////////////////////////
// PRIVATE CHECK
////////////////////////////////////////////////////////////////////////////////

void KociembaSolution::CheckFormat(const char* sSol)
     throw (KociembaException)
{
    // Check if sSol exists
    if (sSol==NULL)
        return;
    
    // Check first pair of brackets
    const char *bo1 = strchr(sSol, '(');
    const char *be1 = strchr(sSol, ')');
    if (bo1==NULL || be1==NULL || bo1>=be1 || bo1!=sSol)
        throw KociembaException(KociembaException::ERR_SOLUTION_PARSE_FORMATINVALID);

    // Check second pair of brackets
    const char *bo2 = strchr(bo1+1, '(');
    const char *be2 = strchr(be1+1, ')');
    if (bo2==NULL || be2==NULL || bo2>=be2 || bo2!=be1+2 || be2!=&sSol[strlen(sSol)-1]
     || strchr(bo2+1, '(') != NULL || strchr(be2+1, ')') != NULL )
        throw KociembaException(KociembaException::ERR_SOLUTION_PARSE_FORMATINVALID);
}
