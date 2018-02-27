/*
	Search problem with UCS and A*
	Author : Samy Aittahar


	### Search problem ###
        Let n,m € N be the size of a grid G € C^(n,m), with C = {'%','G','F', ' ', '.'} and '.' and 'G' which have exactly one occurence in G.

        State space : 
                      X = {(i,j) € N^2 | for all 0 <= i <= n-1, 0 <= j <= m-1}
        Action space : 
                      U = (0,1), (1,0), (-1,0), (0,-1)
        Transition model : 
                      f((x,y),(i,j)) = (max(0,min(x+i,n-1)), max(0,min(x+j,m-1))) 
                                  for all (x,y) € X, (i,j) € U 
        Cost model : c((x,y),(i,j)) = 1 for all (x,y) € X, (i,j) € U
        Goal test : 
		      Given that (x_0,y_0) is NOT the final state,
                      g((x_0,y_0),(i_0,j_0),...,(i_t,j_t),(x_t+1,y_t+1)) (with t € N the size of the path) = satisfied when (x_t+1,y_t+1) = (X,Y) where G[X,Y] = 'G'

	Objective : Design an algorithm which find in a minimum amount of time the least-cost path from any initial state which satisfies the goal test	

        ### Baseline technique ###

        We can simply use UCS, as seen in the project presentation. While it finds, under some particular assumptions, the optimal path, it only uses the information for the current path at each node.
        It implies that more nodes than necessary are often explored. We want to reduce this exploration time while ensuring the optimality of the path

	### Solution ###
        We can reduce the exploration rate by using A* with an admissible fast-computed heuristic function, such as Manhattan distance (absolute distance between two points), 
        which is injected as a-priori knowledge of the search problem. As demonstrated in the execution examples (see at the bottom of this file), A* computation time is faster than UCS, as it needs to explore less nodes to find the optimal path.

        ### Issues and improvements ###
	Current implementation of A* won't scale for largely big grids. Even worse, if the grid is big enough, stack overflow can occurs, especially because of Prolog implementation constraints.
        A solution is to look to 'checkpoints' in order to push away risk of stack overflow. By checkpoints, we means a subset of points for the A* algorithm 
        to include in the path. including the true final state. Limiting depth of A* is an option, using a more sophisticated heuristics to decide which node we should use to initialize the next
        execution of A*. For example, it can be a linear combination between the Manhattan heuristics and a serie of path sampling to approximately look to the least-cost path towards the true final state.
        We do not guarantee anymore the optimality of the path but the solution should be reasonable enough for the decreasing factor of the overall complexity.
        Greedy solvers of Traveling Salesman Person (cross at most once each cell of the grid before reaching final state) can also be used. Cristofides algorithm (https:\/\/en.wikipedia.org\/wiki\/Christofides_algorithm)
        is a greedy solver with bounds on graphs which meet the triangle inequality. Once the maze has been preprocessed in a graph, it's very easy to implement that algorithm and give quickly a reasonable solution in average.

        ### How to use it ###
        1) Declare final state fact like this : final_state(*your final state*)
        2) Declare forbidden state(s) : forbidden_state(*your forbidden state*)
        
*/





/* 
      Utility predicates
*/

%add_list(+L,+L2, -L3) : L3 is the resulting list of pairwise sum of L and L2
add_list([],[],[]).
add_list([X|A],[Y|B],[Z|C]) :- Z is X+Y, add_list(A,B,C).

% sum_list(+L, -C) C is the cumulative sum of the list of numbers L
sum_list([],0).
sum_list([A|L],C) :- sum_list(L,C2), C2 is C + A.

% display(M) : Nice display of the maze M
display([]).
display([L|M]) :- write(L), nl, display(M).
    
% maze(-M,+F) : Load maze M from file indicated by the file path F
maze(M,F) :- open(F, read, Stream), read_maze(Stream,L),close(Stream) ->  
           (convert_to_list_list(L,[[]],MR), reverse_each_list(MR,MR2), reverse(MR2,M)) ; M = [].

% read_maze(+Stream,-M) - M is the list of characters read from Stream

read_maze(Stream,L) :-
    at_end_of_stream(Stream) -> L = [] ; (get_char(Stream,X), read_maze(Stream,M), L = [X|M]).

% reverse_each_list(LL,RL) - RL is the list of reversed list of LL
reverse_each_list([],[]).
reverse_each_list([LR|MR],[L|M]) :- reverse(LR,L), reverse_each_list(MR,M).

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


swap(V,0,K,0,KP,[L|M],[L2|MP]) :- substitute(V,L,K,V2,L3),substitute(V2,L3,KP,V,L2),swap(V,-1,-1,-1,-1,M,MP).
swap(V,0,K,NP,KP,[L|M],[L2|MP]) :- NP>0,NP2 is NP-1,substitute(V,L,K,V2,L2),swap(V2,-1,-1,NP2,KP,M,MP).
swap(V,-1,-1,0,KP,[L|M],[L2|MP]) :- substitute(V,L,KP,V2,L2),swap(V2,-1,-1,-1,-1,M,MP).

swap(V,-1,-1,NP,KP,[L|M],[L|MP]) :- NP>0,NP2 is NP-1,swap(V,-1,-1,NP2,KP,M,MP).
swap(V,-1,-1,-1,-1,[L|M],[L|MP]) :- swap(V,-1,-1,-1,-1,M,MP).




/* 
      Implementation of the components of the search problem
*/


% Available actions (does not change across mazes)
is_action([0,1]).
is_action([0,-1]).
is_action([1,0]).
is_action([-1,0]).

% Transition cost. In the search problem, a cost of 1 is applied everywhere but here we apply a huge cost to the forbidden state in order to stop exploration from that node.
transition_cost(S,N) :- final_state(S) -> N = 10000 ; N = 1.


% is_legal(+[X,Y],+M) - Check if M[X,Y] is not a wall
is_legal([N,K],M) :- nth0(N,M,RM), nth0(K,RM,C), C \= '%'.

% next_state(+X,+A,+M,-XP,-MP,-C) - Applies action A in state X in grid M after ensuring legality. XP is the next state, MP the updated map and C the cost of the transition 
next_state(X,A,M,XP,MP,C) :- is_action(A), add_list(X,A,XP), is_legal(XP,M), swap(X,XP,M,MP), transition_cost(MP,C).





% path_final_state(P) :- Check if the path P ends with the final state, respectively to those who are verified with predicate final_state
path_final_state([S]) :- final_state(S).
path_final_state([S|P]) :- P \= [], \+ forbidden_state(S) ,path_final_state(P).


% play_sequence_action(+I,+As,-P,-TC) - Play the sequence of actions As from initial state I.
play_sequence_action(I,As,P,TC) :- maze(M), get_sequence_state_cost(As,M,[I],0,Ls,TC),reverse(Ls,P),path_final_state(P),write(P). 

get_sequence_state_cost([],_,Ls,Lc,Ls,Lc).
get_sequence_state_cost([A|As],M,[S|L],Lcacc,Ls,Lc) :- next_state(S,A,M,SP,MP,C), Lcacc2 is Lcacc + C, get_sequence_state_cost(As,MP,[SP,S|L],Lcacc2,Ls,Lc).



%Example : 

% play_sequence_action([2,1],[[-1,0],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[1,0]],P,TC).

/* 
      Implementation of A*. Strategy : Start with a node 
      
*/

% Manhattan distance, i.e., |X2 - X1| + |Y2 - Y1|. Final state hard coded.
% heuristic(+S, -H, +TH) : H is Manhattan Distance between S and final state if TH = 1 otherwise H is 0
heuristic([X,Y],0,0).
heuristic([X,Y],H,1) :- final_state([I,J]),A is abs(I - X), B is abs(J - Y), H is A+B.


% Node expansion
% expand(+N, +As, +LN2,-LN) : LN is the concatenation of list of nodes expanded from N with actions As and LN2.
expand([_,_,_,_,_],[],LN,LN).
expand([X,M,La,P,C],[A|As],LN2,LN) :- next_state(X,A,M,XP,MP,C3) -> C2 is C3+C, expand([X,M,La,P,C],As,[[XP,MP,[A|La],[X|P],C2]|LN2],LN) ; expand([X,M,La,P,C],As,LN2,LN).

% get_final_path_node(+LN, -N) - N is in LN and contains the final state 

get_final_path_node([[X,M,La,P,C] | _],[X,M,La,P,C]) :- final_state(S), P = [S|_].
get_final_path_node([[_,_,_,P,_] | L],N) :-  final_state(S), \+P = [S|_], get_final_path_node(L,N).

% greedy_get_node(+LN, +HC, +N2, -LN2, -N) - N is N2 or the least cost node of value HC in LN, LN2 is LN without the least cost node of value HC.  
greedy_get_node([],HN,_,N,[],N).
greedy_get_node([NC|LN],HN,HC,N2,[N2|LN2],N) :- NC = [X,_,_,_,C], heuristic(X,H,HN), HC2 is H+C, HC2 < HC, greedy_get_node(LN,HC2,NC,LN2,N).
greedy_get_node([NC|LN],HN,HC,N2,[NC|LN2],N) :- NC = [X,_,_,_,C], heuristic(X,H,HN), HC2 is H+C, HC2 >= HC, greedy_get_node(LN,HC,N2,LN2,N).

% astar(+X, +M, +As, +H, -P, -C) : P is the optimal path from X to the final state using the components of the search problem described above, given that the used heuristic H is admissible (i.e. between h0 and h*). C is the optimal cost.
astar(X, M, As, H, P, C) :- astar([[X,M,[],[],0]], As, H, [_,_,_,P2,C]), reverse(P2,P).

% astar(+LN, +As, +H, -NF) - NF is the final node contained in LN. If LN does not contains a final node, the least sum heuristic (H) +cost node is expanded. Fail if there is no final or expandable node in LN.
astar(LN,As, HN,NF) :- get_final_path_node(LN, N) -> N = NF ; LN = [N2 | LN4], N2 = [X,_,_,_,C],heuristic(X,H,HN), HC is H+C,greedy_get_node(LN4,HN,HC,N2,LN2,N3),expand(N3,As,LN2,LN3), astar(LN3,As,NF).


/*
	Execution examples
	
	maze(M,"maze.txt"), astar([2,1],M,[[0,1],[1,0],[-1,0],[0,-1]], P, C).

*/
%Example : 
%  
