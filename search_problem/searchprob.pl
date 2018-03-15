/*

        Note for XXX : The current implementation handles search in a classic multi-target maze.
        For the Aperture maze, you have to modify the environment components to handle portals (creation and traversal).  

*/


/*
        Search problem with UCS and A*
        Author : Samy Aittahar


        ### Search problem ###
        Let n,m € N be the size of a grid G € C^(n,m), with C = {'%','G','F', ' ', '.'} and '.' which have exactly one occurence in G.

        State space : 
                      X = {(i,j) € N^2 | for all 0 <= i <= n-1, 0 <= j <= m-1}
        Action space : 
                      U = (0,1), (1,0), (-1,0), (0,-1)
        Transition model : 
                      f((x,y),(i,j)) = (max(0,min(x+i,n-1)), max(0,min(x+j,m-1))) 
                                  for all (x,y) € X, (i,j) € U 
        Cost model : [(x',y') = f((x,y),(i,j))] - c((x,y),(i,j)) = 10 if G[x',y'] = 'G', +inf if G[x',y'] = 'F', 1 otherwise.
        Goal test : 
                      g((x_0,y_0),(i_0,j_0),...,(i_t,j_t),(x_t+1,y_t+1)) is satisfied when 
                      (i) for all 0 <= k <= t, (x_{k+1},y_{k+1}) = f((x_k,y_k),(i_k,j_k)), 
                      (ii) for all (x,y) € X, G[x,y] = 'G' => (x,y) is in (x_0,y_0),(i_0,j_0),...,(i_t,j_t),(x_t+1,y_t+1),
                      and (iii) for all 0 <= k <= t+1, G[x,y] =\/= 'F'  

        Objective : Design an algorithm which find in a minimum amount of time the least-cost path from any initial state which satisfies the goal test

        ### Baseline technique ###

        As seen in the presentation, A* seems to be a good candidate, with the Min Manhattan distance. The Min Manhattan distance does only consider the closest point of interest (here, the 'G's). However, this could result in Prolog as a stack overflow when the grid is not that big enough. We propose below a solution to overcome this issue and have a better scale of A* while tolerating a suboptimality on the computed path.   

        ### Solution ###
        
        We propose to break up the searching process into several stage. At each stage, we run A* with Min Manhattan distance, which stops when a 'G' state has been picked by the algorithm. Initially, the first stage starts with the initial state of the problem. The next stage starts with the last state of the current stage. The whole path is the concatenation of those who have been computed through these stages. To do so, we choose a node using the Min Manhattan distance. Results shows that this approach works well on "dense" grids (e.g. with many targets) but works poorly on sparse grids. We describe below possible improvements to overcome this issue.

        ### Improvements ###
       
        - Limitation of the searching depth. The chosen node is the one who minimize the sum C+H, where C is the cost of the path so far and H the Min Manhattan Distance.
           - H could be changed to prefer a more sophisticated heuristic (random suffix path towards G, ant colony heuristic...)
        - Pruning states on some criterion (already visited for example) 
        - Not requested here but the memory occupancy could be improved when A* is executed. Indeed, while the implementation is more readable here, there a common path through nodes that are kept in memory. Maybe we could separate somewhere this common path to not repeat it ?
      
*/





/* 
      Utility predicates
*/

% add_list(+L,+L2, -L3) : L3 is the resulting list of pairwise sum of L and L2
add_list([],[],[]).
add_list([X|A],[Y|B],[Z|C]) :- Z is X+Y, add_list(A,B,C).

% sum_list(+L, -C) C is the cumulative sum of the list of numbers L
sum_list([],0).
sum_list([A|L],C) :- sum_list(L,C2), C2 is C + A.
    
% maze(-M,+F) : Load maze M from file indicated by the file path F
maze(M,F) :- open(F, read, Stream), read_maze(Stream,L),close(Stream) ->  
           (convert_to_list_list(L,[[]],MR), reverse_each_list(MR,MR2), myreverse(MR2,M)) ; M = [].

/*

        Execution example of maze(M,F) : 

        ?- maze(M,"maze.txt").
        M = 
        [['%', '%', '%', '%', '%', '%', '%', '%'], 
         ['%', 'G', ' ', ' ', ' ', ' ', ' ', '%'], 
         ['%', ' ', '%', '%', '%', ' ', ' ', '%'], 
         ['%', '.', '%', 'G', ' ', ' ', ' ', '%'], 
         ['%', ' ', '%', '%', '%', ' ', ' ', '%'], 
         ['%', ' ', ' ', ' ', ' ', ' ', 'G', '%'], 
         ['%', '%', '%', '%', '%', '%', '%', '%']]

*/

% read_maze(+Stream,-M) - M is the list of characters read from Stream

read_maze(Stream,L) :-
    at_end_of_stream(Stream) -> L = [] ; (get_char(Stream,X), read_maze(Stream,M), L = [X|M]).

% reverse_each_list(LL,RL) - RL is the list of reversed list of LL
reverse_each_list([],[]).
reverse_each_list([LR|MR],[L|M]) :- myreverse(LR,L), reverse_each_list(MR,M).

% convert_to_list_list(L,M,M2) - M2 is the concatenation of the list of list extracted from string L with \n as separator and M
    
convert_to_list_list([],M,M).
convert_to_list_list(['\n'],M,M).
convert_to_list_list(['\n'|L],L2,M) :- L \= [],convert_to_list_list(L,[[]|L2],M).
convert_to_list_list([X|L],[SL|L2],M) :- X \= '\n', L \= [],convert_to_list_list(L,[[X|SL]|L2],M).

%substitute(+V,+L,+I,-V2,-L2) : L2 is L with Ith cell content replaced by V. The original content is returned via V2
substitute(V,[],-1,V,[]).
substitute(V,[X|L],KP,V2,[X|L2]) :- KP>0,KP2 is KP-1, substitute(V,L,KP2,V2,L2).
substitute(V,[X|L],0,V2,[V|L2]) :- substitute(X,L,-1,V2,L2).
substitute(V,[X|L],-1,V2,[X|L2]) :- substitute(V,L,-1,V2,L2).

%swap(+S1,+S2,+M,-MP) : MP is M with content in S1 and S2 exchanged in their respective cells

swap([N,K],[NP,KP],M,MP) :- N=<NP, swap(' ',N,K,NP,KP,M,MP).
swap([N,K],[NP,KP],M,MP) :- N>NP, swap('.',NP,KP,N,K,M,MP).

%Auxiliary predicates using pseudo accumulators to facilitate the swapping between two cells in M (i.e. to execute swap predicate above)
swap(_,-1,-1,-1,-1,[],[]).
swap(V,N,K,NP,KP,[L|M],[L|MP]) :- N>0,NP>0,NP2 is NP-1,N2 is N-1,swap(V,N2,K,NP2,KP,M,MP).


swap(V,0,K,0,KP,[L|M],[L2|MP]) :- substitute(V,L,K,V2,L3),substitute(V2,L3,KP,_,L2),swap(_,-1,-1,-1,-1,M,MP).
swap(V,0,K,NP,KP,[L|M],[L2|MP]) :- NP>0,NP2 is NP-1,substitute(V,L,K,V2,L2),swap(V2,-1,-1,NP2,KP,M,MP).
swap(V,-1,-1,0,KP,[L|M],[L2|MP]) :- substitute(V,L,KP,V2,L2),swap(V2,-1,-1,-1,-1,M,MP).

swap(V,-1,-1,NP,KP,[L|M],[L|MP]) :- NP>0,NP2 is NP-1,swap(V,-1,-1,NP2,KP,M,MP).
swap(V,-1,-1,-1,-1,[L|M],[L|MP]) :- swap(V,-1,-1,-1,-1,M,MP).

/*

        This is an acceptable implementation for this project, but this function is not generic enough. 
        Swap is supposed to be a bubble exchange function. But doing this would require to make a double traversal of the matrix.
        Swapping is done a lot in A* traversal, so we prefer to keep this implementation to ensure that only a traversal is done.
        Moreover, we know that '.' will always left a space in his current cell after reaching another cell.
        So we recommend to use swap only to move '.' (i.e. the first argument is always the location of '.') towards another legal point (not a wall or a forbidden state) and nothing else.

        Execution examples : 
        ?- maze(M,"maze.txt"), swap([3,1],[4,1],M,MP), swap([4,1],[3,1],MP,M).
 
        M = [['%', '%', '%', '%', '%', '%', '%', '%'], 
             ['%', 'G', ' ', ' ', ' ', ' ', ' ', '%'], 
             ['%', ' ', '%', '%', '%', ' ', ' ', '%'], 
             ['%', '.', '%', 'G', ' ', ' ', ' ', '%'], 
             ['%', ' ', '%', '%', '%', ' ', ' ', '%'], 
             ['%', ' ', ' ', ' ', ' ', ' ', 'G', '%'], 
             ['%', '%', '%', '%', '%', '%', '%', '%']],

        MP = [['%', '%', '%', '%', '%', '%', '%', '%'], 
              ['%', 'G', ' ', ' ', ' ', ' ', ' ', '%'], 
              ['%', ' ', '%', '%', '%', ' ', ' ', '%'], 
              ['%', ' ', '%', 'G', ' ', ' ', ' ', '%'], 
              ['%', '.', '%', '%', '%', ' ', ' ', '%'], 
              ['%', ' ', ' ', ' ', ' ', ' ', 'G', '%'], 
              ['%', '%', '%', '%', '%', '%', '%', '%']] 
      

        
        

*/

% remove(+Elem, +L, -L2) : L2 is L without Elem
remove(_,[],[]).
remove(X,[X|L2],L) :- remove(X,L2,L). 
remove(X,[Y|L2],[Y|L]) :- X\=Y, remove(X,L2,L). 

% not_member(+X, -L) : True if X is not in L
not_member(_,[]).
not_member(X,[Y|L]) :- X \= Y; not_member(X,L).

% intersect(+A,+B,-C) : C is the list of elements that are both in A and B (intersection)
intersect([],_,[]).
intersect([A|As],Bs,Cs1) :-
    intersect(As,Bs,Cs), (member(A,Bs) -> Cs1 = [A|Cs], Cs1 = Cs).

% fast_intersect(+A,+B,-C) : if A and B are sorted with the same ordering rule, C is the list of elements that are both in A and B (intersection)
fast_intersect([],_,[]).
fast_intersect(_,[],[]).
fast_intersect([A|L],[A|L2],[A|L3]) :- fast_intersect(L,L2,L3).
fast_intersect([A|L],[B|L2],L3) :- B\=A,fast_intersect([A|L],L2,L3).

% coords_symbols(+M,+S,-L) : L is the list of 2D coordinates of a list of list M where S appears
coords_symbol(M,S,L) :- coords_symbol(M,S,0,[],L).

% Auxiliary predicate for coord_symbols in the first dimension of M, looking for the Nth member of M
coords_symbol([],_,_,L,L).
coords_symbol([R|M],S,N,Lacc,L) :- N2 is N+1, coords_symbol(R,S,N,0,Lacc,L2),coords_symbol(M,S,N2,L2,L).

% Auxiliary predicate for coord_symbols in the second dimension of M, looking for the Mth member of a list R
coords_symbol([],_,_,_,L,L).
coords_symbol([S|R],S,N,M,LAcc,L) :- M2 is M+1, coords_symbol(R,S,N,M2,[[N,M]|LAcc],L).
coords_symbol([S2|R],S,N,M,LAcc,L) :- S2\=S, M2 is M+1, coords_symbol(R,S,N,M2,LAcc,L).

% myreverse(+L,-R) : R is L reversed
myreverse(L,R) :- myreverse(L,[],R).

% Auxiliary predicate for myreverse with an accumulator list
myreverse([],L,L).
myreverse([A|L], LAcc, L2) :- myreverse(L,[A|LAcc], L2).

% flatten_states(+L,-L2) : L2 is the list of pairs [A,B] in L where A and B are integer
flatten_states(L,L2) :- flatten_states(L,[],L2).

% Auxiliary predicate for flatten_states with an accumulator list
flatten_states([],L,L).
flatten_states([A|L], L2, [A|L3]) :- A = [B,C], integer(B), integer(C), flatten_states(L,L2,L3), !.
flatten_states([L1|L2], L3, L4) :- flatten_states(L2,L3,L5), flatten_states(L1,L5,L4). 
/* 
      Implementation of the components of the search problem
*/


% Available actions (does not change across mazes)

/* 
      XXX : You'll have to consideralso shooting actions for the Aperture maze
*/

is_action([0,1]).
is_action([0,-1]).
is_action([1,0]).
is_action([-1,0]).


% Evaluation of a symbol
eval_cell('G',0).
% Unless there is no path than crossing a F, this score is enough to prevent the traversal of such state
eval_cell('F',10000).
eval_cell(C,10) :- not_member(C,['G','F']).
% eval_state(+M, +S, -N) : N is the immediate score signal of the cell located at 2D coordinates S in the list of list M 
eval_state(M,[X,Y],N) :- nth0(X,M,L), nth0(Y,L,C), eval_cell(C,N).


% is_legal(+[X,Y],+M) - True if M[X,Y] is a legal state (here, not a wall)
is_legal([N,K],M) :- nth0(N,M,RM), nth0(K,RM,C), C \= '%'.

% next_state(+X,+A,+M,-XP,-MP,-C) - Applies action A in state X in grid M after ensuring legality. XP is the next state, MP the updated map and C the cost of the transition. 
% M[X] must be the current location of the dot 
next_state(X,A,M,XP,MP,C) :- is_action(A), add_list(X,A,XP), is_legal(XP,M), eval_state(M,XP,C), swap(X,XP,M,MP).





% path_final_state(+P,-LSF) : Check if the path P contains all the desired states, at least once.
path_final_state(P,LSF) :- intersect(LSF,P,LSF).

% fast_path_final_state(P,LSF) : Check if the path P contains all the desired states, at least once. Assume that P and LSF are lexicographically sorted
% Not used here because we do not sort P in A*
fast_path_final_state(P,LSF) :- fast_intersect(LSF,P,LSF).




%Example : 

% play_sequence_action([2,1],[[-1,0],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[1,0]],P,TC).

/* 
      Implementation of A*. 
      A node is a 5-uplet composed of the current state, the current map, the list of actions done so far (reversed), the current sequence of state so far(reversed), and the current cumulated cost so far.
      It start with an initial node with the initial state, the initial map, an empty list of actions, an empty list of states and a cost of 0
      Additional inputs are final states, automatically fetched in the map.
      Expansion is done recursively until A* pick one of the final states. Then it stops and start again A*, which stops when it picks another final state, and so on, until there is no final state left.
      Big issue is sparse grids, with stack overflow. XXX will have to handle this case in the Aperture Maze, by maybe starting with the current implementation before going with the latter.
      Some suggestions are made at the head of this file.
      
*/

% Manhattan distance, i.e., |X2 - X1| + |Y2 - Y1|. 
% m_distance(+S, +S2, +M) : M is Manhattan Distance between S and S2.
m_distance([X,Y],[V,Z],M) :- A is abs(V - X), B is abs(Z - Y), M is A+B. 

% min_m_distance(+S, +L, +M) : M is the Manhattan Distance between S and argmin_SD MD(S,SD) where MD is the manhattan distance function and SD is in L.
min_m_distance(S,L,A) :- min_m_distance(S, L, 100000, A). 

% Auxiliary predicate for min_m_distance with a pseudo accumulator to keep the current minimum manhattan distance
min_m_distance(_,[],M,M).
min_m_distance(S,[S2|L],A,M) :- m_distance(S,S2,C), (C < A -> min_m_distance(S,L,C,M); min_m_distance(S,L,A,M)).

% heuristic(+S,+L,+H,-V) : V is 0 if H is 0 and V is the minimum manhattan distance between state S and the list of state LSF if H is 1
heuristic(_,_,0,0).
heuristic(S,LSF,1,H) :- min_m_distance(S,LSF,H).


% expand(+N, +As, +LN2,-LN) : LN is the concatenation of list of legal successors nodes from N expanded with the list of actions As and LN2.
expand(_,[],LN,LN).
expand([X,M,La,P,C],[A|As],LN2,LN) :- next_state(X,A,M,XP,MP,C3) -> C2 is C3+C, expand([X,M,La,P,C],As,[[XP,MP,[A|La],[X|P],C2]|LN2],LN) ; expand([X,M,La,P,C],As,LN2,LN).


% greedy_get_node(+LN, +LSF, +HN, +H2, +N2, -LN2, -N) - N is N2 if H2 is the minimum sum of cost + heuristic, otherwise it is the node in LN which exhibit the minimum sum of cost + heuristic. LN2 is LN without the node which exhibit the minimum sum of cost + heuristic.  
greedy_get_node([],_,_,_,N,_,N).
greedy_get_node([NC|LN],LSF,HN,H2,N2,[N3|LN2],N) :- NC = [X,_,_,_,C], heuristic(X,LSF,HN,H), HC is H+C, (HC < H2 -> N3 = N2,greedy_get_node(LN,LSF,HN,HC,NC,LN2,N);N3 = NC,greedy_get_node(LN,LSF,HN,H2,N2,LN2,N)).

% astar(+M,+As,+H,-P,-C) : P is the path satisfying the goal test from the initial state defined in M with available actions in As. C is the total cost of P. UCS is executed if H = 0 and A* is executed with min manhattan distance if H = 1. 
astar(M,As,H,P,C) :- coords_symbol(M,'G',LSF), coords_symbol(M,'.',[X|_]), multistage_astar(X,LSF,M,As,H,[],P2,C), myreverse(P2,P3), flatten_states(P3,P).

/*
        Execution examples (paths are suboptimal here)
        
        ?- maze(M,"maze.txt"), findall(X,is_action(X),As), astar(M,As,1,P,C).
        M = [['%', '%', '%', '%', '%', '%', '%', '%'], 
             ['%', 'G', ' ', ' ', ' ', ' ', ' ', '%'], 
             ['%', ' ', '%', '%', '%', ' ', ' ', '%'], 
             ['%', '.', '%', 'G', ' ', ' ', ' ', '%'], 
             ['%', ' ', '%', '%', '%', ' ', ' ', '%'], 
             ['%', ' ', ' ', ' ', ' ', ' ', 'G', '%'], 
             ['%', '%', '%', '%', '%', '%', '%', '%']],
        As = [[0, 1], [0, -1], [1, 0], [-1, 0]],
        P = [[3, 1], [2, 1], [1, 1], [1, 2], [1, 3], [1, 4], [1, 5], [2, 5], [3, 5], [3, 4], [3, 3], [3, 4], [3, 5], [4, 5], [5, 5]],
        C = 120

        ?- maze(M,"bigmaze.txt"), findall(X,is_action(X),As), astar(M,As,1,P,C).
        M = [['%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%'], 
             ['%', 'G', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'G', ' ', ' ', ' ', ' ', 'G', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '%'], 
             ['%', ' ', '%', '%', '%', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '%'], 
             ['%', '.', '%', 'G', ' ', ' ', ' ', ' ', ' ', ' ', 'G', ' ', ' ', ' ', ' ', ' ', 'G', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'G', ' ', ' ', ' ', 'G', '%'], 
             ['%', ' ', '%', '%', '%', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '%'], 
             ['%', ' ', ' ', ' ', ' ', ' ', 'G', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'G', ' ', ' ', ' ', ' ', 'G', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '%'], 
             ['%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%', '%']],
        As = [[0, 1], [0, -1], [1, 0], [-1, 0]],
        P = [[3, 1], [2, 1], [1, 1], [2, 1], [3, 1], [4, 1], [5, 1], [5, 2], [5, 3], [5, 4], [5, 5], [5, 6], [4, 6], [3, 6], [3, 5], [3, 4], [3, 3], [3, 4], [3, 5], [3, 6], [3, 7], [3, 8], [3, 9], [3, 10], [3, 11], [3, 12], [3, 13], [3, 14], [3, 15], [3, 16], [3, 17], [4, 17], [4, 18], [4, 19], [4, 20], [5, 20], [4, 20], [3, 20], [2, 20], [1, 20], [1, 21], [1, 22], [1, 23], [1, 24], [1, 25], [2, 25], [3, 25], [4, 25], [5, 25], [4, 25], [4, 26], [3, 26], [3, 27], [3, 28], [3, 29], [3, 30], [3, 31], [3, 32]],
        C = 460 

        

        ?- maze(M,"sparsemaze.txt"), findall(X,is_action(X),As), astar(M,As,1,P,C).
        Out of global stack (See at the head of this file for explanations)

*/

% Auxiliary predicate  to loop over several executions of astar. It stops when there is no remaining target state. It uses a list accumulator to keep all the subpaths. 
multistage_astar(X,[W],M,As,H,PAcc,[P2|PAcc],C2) :- astar(X,[W],M,As,H,P3,W,_,C2), myreverse(P3,P2).
multistage_astar(X,[W,W2|LSF],M,As,H,PAcc,P,C) :- astar(X,[W,W2|LSF],M,As,H,P3,XF,MF,C2), myreverse(P3,P2), remove(XF,[W,W2|LSF],LSF2), multistage_astar(XF,LSF2,MF,As,H,[P2|PAcc],P,C4), C is C2+C4.


 

% astar(+X,+LSF, +M, +As, +H, -P, -XF, -MF, -C) : Auxiliary predicate for astar. P is a path from X to one of the target states LSF in the maze M. XF is this state and MF is the updated map. C is the cost of path P. H determines if UCS or A* with min manhattan distance is used.
astar(X, LSF, M, As, H, P, XF, MF, C) :- astar_aux([[X,M,[],[],0]], LSF,As, H, [XF,MF,_,P,C]).

% astar(+LN, +LSF,+As, +H, -NF) - Auxiliary predicate for astar. H determines if astar or UCS is executed for node selection. LN is recursively updated by expanding the least sum of cost + heuristic with As. This predicate ends when the final state NF is picked from LN following chosen rule. 
astar_aux([N2 | LN4],LSF,As, HN,NF) :- N2 = [X,_,_,_,C],heuristic(X,LSF,HN,H), HC is H+C,greedy_get_node(LN4,LSF,HN,HC,N2,LN2,[X2|Xs]), (member(X2,LSF) -> NF = [X2|Xs] ; expand([X2|Xs],As,LN2,LN3), astar_aux(LN3,LSF,As,HN,NF)), !.



