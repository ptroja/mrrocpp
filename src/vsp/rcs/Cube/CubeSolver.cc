/////////////////////////////////////////////////////////
// Name: CubeSolver.cpp
// Implements: CubeSolver
/////////////////////////////////////////////////////////


#include "vsp/rcs/Cube/CubeSolver.h"
#include <string>
#include <stdio.h>


#define RET_SUCCESS   0
#define RET_FAILURE   1

#define HUGE_INT 10000
#define SEARCH_LIMIT 10000000


bool CubeSolver::bLog = false;
bool CubeSolver::bTest = false;


////////////////////////////////////////////////////////////////////////////////
// SOLVE PUBLIC METHOD and its private helpers
////////////////////////////////////////////////////////////////////////////////

void CubeSolver::Solve(Cube* scrambledCube, const CubeSolution& solution)
     throw (Exception)
{
     int i, len = solution.GetLength();
     CubeMove::eMove move;
     CubeMove::eTurn turn;

     for (i=0; i<len; i++)
     {
         solution.GetMove(i, move, turn);
         scrambledCube->Move(move, turn);
     }
}

CubeSolution* CubeSolver::FindSolution(Cube* scrambledCube)
    throw (Exception)
{
    int result = RET_FAILURE;
    
    // initialize solver before finding solution
    Init();
    
    // initiate context of solver
    CubeSolverContext context;

    // clone object to be solved
    Cube *pCube = scrambledCube->Clone();

    // estimate cost of solution
    context.iThreshold = Heuristic(pCube);

    // return empty solution if object is already solved
    if (context.iThreshold == 0) 
    {
        CubeSolution *ret_sol = new CubeSolution();
        ret_sol->SetInfo(CubeSolution::INFO_ALREADY_SOLVED, true);
        delete pCube;
        return ret_sol;
    }
        
    // iterate search of solution as long as one not found
    for (context.iIteration = 1; result == RET_FAILURE; context.iIteration++)
    {
        if (bLog) printf("Search iteration %d to depth %d\n", 
           context.iIteration, context.iThreshold);

        // set maximum depth of next iteration
        context.iThresholdNext = HUGE_INT;

        // Perform the recursive IDA* search
        context.iNodesExpanded = 0;
        context.iNodesChecked = 0;
        result = Search(context, pCube, context.iThreshold);
        context.iNodesExpandedAll += context.iNodesExpanded;
        context.iNodesCheckedAll += context.iNodesChecked;
        
        // Establish a new threshold for a deeper search
        context.iThreshold = context.iThresholdNext;
            
        if (bLog) printf("Searched through %d nodes.\n", context.iNodesExpanded);
    } 
    
    // free used memory
    delete pCube;
    
    // print information for tests if requested
    if (bTest)
    {
        printf("%d;%d;%d;%d;%d;%d;%d;", context.solution.GetLength(),
            context.iIteration-1, context.iThreshold,
            context.iNodesExpanded, context.iNodesExpandedAll,
            context.iNodesChecked, context.iNodesCheckedAll);
    }

    // return found solution and clear pointer to it in solver
    CubeSolution *ret_sol = new CubeSolution(context.solution);
    return ret_sol;
}

CubeSolution* CubeSolver::FindNextSolution(Cube* scrambledCube, const CubeSolution& solution)
    throw (Exception)
{
    int result = RET_FAILURE;

    if (solution.GetLength() == 0)
        return FindSolution(scrambledCube);

    // initialize solver before finding solution
    Init();
    
    // initiate context of solver
    CubeSolverContext context(solution);
    
    // clone object to be solved
    Cube *pCube = scrambledCube->Clone();

    // estimate cost of solution
    context.iThreshold = solution.GetLength(); 

    // return empty solution if object is already solved
    if (context.iThreshold == 0) 
    {
        CubeSolution *ret_sol = new CubeSolution();
        ret_sol->SetInfo(CubeSolution::INFO_ALREADY_SOLVED, true);
        delete pCube;
        return ret_sol;
    }
      
    // iterate search of solution as long as one not found
    for (context.iIteration = 1; result == RET_FAILURE; context.iIteration++)
    {
        if (bLog) printf("Search iteration %d to depth %d\n", 
            context.iIteration, context.iThreshold);

        // set maximum depth of next iteration
        context.iThresholdNext = HUGE_INT;

        // Perform the recursive IDA* search
        context.iNodesExpanded = 0;
        context.iNodesChecked = 0;
        result = SearchNext(context, pCube, context.iThreshold);
        context.iNodesExpandedAll += context.iNodesExpanded;
        context.iNodesCheckedAll += context.iNodesChecked;
            
        // Establish a new threshold for a deeper search
		if (context.iMinSolLength > 0)
		{
			context.iThreshold++;
            context.iMinSolLength = context.iThreshold;
		}
		else
			context.iThreshold = context.iThresholdNext;
            
        if (bLog) printf("Searched through %d nodes.\n", context.iNodesExpanded);
    } 
   
    // free used memory
    delete pCube;

    // return found solution and clear pointer to it in solver
    CubeSolution *ret_sol = new CubeSolution(context.solution);
    return ret_sol;
}
    
int CubeSolver::Search(CubeSolverContext& context, Cube *cube, int parent_totalcost) 
{
    int result;
        
    // get estimated cost of solution from this point
    int cost = Heuristic(cube);  // h
        
    // increase number of checked nodes
    context.iNodesChecked++;

    // solution found if estimated cost is 0
    if (cost == 0) 
        return RET_SUCCESS;
        
    // get estimated cost of whole solution
    int totalCost = ( ( (context.iDepth + cost) > parent_totalcost ) ?  
        (context.iDepth + cost) : (parent_totalcost)); // f' = max(f, g'+h')
        
    int mov, tur;
    CubeMove::eMove move;
    CubeMove::eTurn turn;

    // if node is in current area of search
    if (totalCost <= context.iThreshold) 
    {
        // increase number of expanded nodes
        context.iNodesExpanded++;

        // expand node - with each move
        for (mov=0; mov<CubeMove::MOVE_NUMBER; mov++) 
        {
            move = (CubeMove::eMove) mov;

            // set move with default turn, check if solution allowed
            context.solution.SetMove(context.iDepth, move, CubeMove::QUARTER);
            if (!context.solution.IsAllowed())
                continue;
            
            // expand node - with each turn of move
            for (tur=0; tur<CubeMove::TURN_NUMBER; tur++) 
            {
                turn = (CubeMove::eTurn) tur;
                
                // check if move is allowed on cube
                if (cube->IsAllowed(move, turn))
                {
                    // set move and turn and perform this move on cube
                    context.solution.SetMove(context.iDepth, move, turn);
                    cube->Move(move, turn);

                    // search deeper
                    context.iDepth++;
                    result = Search(context, cube, totalCost);
                    context.iDepth--;
                    
                    // exit on success
                    if (result != RET_FAILURE)
                        return result;

                    // undo the move
                    cube->Move(move, CubeMove::REVERSE_TURN(turn));
                }
            }
        }
    }  
    
    else 
    {
        // minimally enlarge search area
        if (totalCost < context.iThresholdNext) 
            context.iThresholdNext = totalCost;
    }
    
    return RET_FAILURE;
}

int CubeSolver::SearchNext(CubeSolverContext& context, Cube *cube, int parent_totalcost) 
{
    int result;

    // get estimated cost of solution from this point
    int cost = Heuristic(cube);  // h
        
    // increase number of checked nodes
    context.iNodesChecked++;

    // solution found if estimated cost is 0
    if (cost == 0 && (context.iMinSolLength == 0 || context.iDepth >= context.iMinSolLength))
        return RET_SUCCESS;
        
    // get estimated cost of whole solution
    int totalCost = ( ( (context.iDepth + cost) > parent_totalcost ) ?  
        (context.iDepth + cost) : (parent_totalcost)); // f' = max(f, g'+h')
        
    int mov, tur;
    CubeMove::eMove move, solMove;
    CubeMove::eTurn turn, solTurn;

    // if node is in current area of search
    if (totalCost <= context.iThreshold) 
    {
        // increase number of expanded nodes
        context.iNodesExpanded++;

        // expand node - with each move
        for (mov=0; mov<CubeMove::MOVE_NUMBER; mov++) 
        {
            move = (CubeMove::eMove) mov;

            // for next solution
            if (context.bNext)
            {
                context.solution.GetMove(context.iDepth, solMove, solTurn);
                if (move != solMove)
                    continue;
            }
            else
            {
                // set move with default turn, check if solution allowed
                context.solution.SetMove(context.iDepth, move, CubeMove::QUARTER);
                if (!context.solution.IsAllowed())
                    continue;
            }
            
            // expand node - with each turn of move
            for (tur=0; tur<CubeMove::TURN_NUMBER; tur++) 
            {
                turn = (CubeMove::eTurn) tur;
                
                // for next solution
                if (context.bNext)
                {
                    context.solution.GetMove(context.iDepth, solMove, solTurn);
                    if (turn != solTurn)
                        continue;
                    if (context.iDepth + 1 == context.solution.GetLength())
                    {
                        context.bNext = false;
                        continue;
                    }
                }
                
                // check if move is allowed on cube
                if (cube->IsAllowed(move, turn))
                {
                    // set move and turn and perform this move on cube
                    if (!context.bNext)
                        context.solution.SetMove(context.iDepth, move, turn);
                    cube->Move(move, turn);

                    // search deeper
                    context.iDepth++;
                    result = SearchNext(context, cube, totalCost);
                    context.iDepth--;
                    
                    // exit on success
                    if (result != RET_FAILURE)
                        return result;

                    // undo the move
                    cube->Move(move, CubeMove::REVERSE_TURN(turn));
                }
            }
        }
    }  
    
    else 
    {
        // minimally enlarge search area
        if (totalCost < context.iThresholdNext) 
            context.iThresholdNext = totalCost;
    }
    
    return RET_FAILURE;
}
