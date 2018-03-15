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

        As seen in the presentation, A* seems to be a good candidate, with the Min Manhattan distance. The Min Manhattan distance does only consider the closest point of interest (here, the 'G's). However, we need our algorithm to scale in big grids. We propose below a modification of the existing A* algorithm with a trade-off between the quality of the solution and the tractability.  

	### Solution ###
	
	We propose to break up the searching process into several stage. A stage ends when the closest 'G' has been reached. To do so, we choose a node using the Min Manhattan distance

        
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
           (convert_to_list_list(L,[[]],MR), reverse_each_list(MR,MR2), myreverse(MR2,M)) ; M = [].

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

remove(_,[],[]).
remove(X,[X|L2],L) :- remove(X,L2,L). 
remove(X,[Y|L2],[Y|L]) :- X\=Y, remove(X,L2,L). 

not_member(_,[]).
not_member(X,[Y|L]) :- X \= Y; not_member(X,L).

intersect([],_,[]).
intersect([A|As],Bs,Cs1) :-
    intersect(As,Bs,Cs), (member(A,Bs) -> Cs1 = [A|Cs], Cs1 = Cs).

fast_intersect([],_,[]).
fast_intersect(_,[],[]).
fast_intersect([A|L],[A|L2],[A|L3]) :- fast_intersect(L,L2,L3).
fast_intersect([A|L],[B|L2],L3) :- B\=A,fast_intersect([A|L],L2,L3).

coords_symbol(M,S,L) :- coords_symbol(M,S,0,[],L).

coords_symbol([],_,_,L,L).
coords_symbol([R|M],S,N,Lacc,L) :- N2 is N+1, coords_symbol(R,S,N,0,Lacc,L2),coords_symbol(M,S,N2,L2,L).


coords_symbol([],_,_,_,L,L).
coords_symbol([S|R],S,N,M,LAcc,L) :- M2 is M+1, coords_symbol(R,S,N,M2,[[N,M]|LAcc],L).
coords_symbol([S2|R],S,N,M,LAcc,L) :- S2\=S, M2 is M+1, coords_symbol(R,S,N,M2,LAcc,L).

myreverse(L,R) :- myreverse(L,[],R).

myreverse([],L,L).
myreverse([A|L], LAcc, L2) :- myreverse(L,[A|LAcc], L2).

flatten_states(L,L2) :- flatten_states(L,[],L2).

flatten_states([],L,L).
flatten_states([A|L], L2, [A|L3]) :- A = [B,C], integer(B), integer(C), flatten_states(L,L2,L3), !.
flatten_states([L1|L2], L3, L4) :- flatten_states(L2,L3,L5), flatten_states(L1,L5,L4). 
/* 
      Implementation of the components of the search problem
*/


% Available actions (does not change across mazes)
is_action([0,1]).
is_action([0,-1]).
is_action([1,0]).
is_action([-1,0]).

% Transition cost. In the search problem, a cost of 1 is applied everywhere but here we apply a huge cost to the forbidden state in order to stop exploration from that node.
eval_cell('G',0).
eval_cell('F',10000).
eval_cell(C,10) :- not_member(C,['G','F']).
eval_state(M,[X,Y],N) :- nth0(X,M,L), nth0(Y,L,C), eval_cell(C,N).


% is_legal(+[X,Y],+M) - Check if M[X,Y] is not a wall
is_legal([N,K],M) :- nth0(N,M,RM), nth0(K,RM,C), C \= '%'.

% next_state(+X,+A,+M,-XP,-MP,-C) - Applies action A in state X in grid M after ensuring legality. XP is the next state, MP the updated map and C the cost of the transition 
next_state(X,A,M,XP,MP,C) :- is_action(A), add_list(X,A,XP), is_legal(XP,M), eval_state(M,XP,C), swap(X,XP,M,MP).





% path_final_state(P,LSF) : Check if the path P contains all the desired states, at least once.
path_final_state(P,LSF) :- intersect(LSF,P,LSF).

% fast_path_final_state(P,LSF) : Check if the path P contains all the desired states, at least once. Assume that P and LSF are lexicographically sorted
fast_path_final_state(P,LSF) :- fast_intersect(LSF,P,LSF).


% play_sequence_action(+I,+As,-P,-TC) - Play the sequence of actions As from initial state I.
play_sequence_action(I,As,P,TC) :- maze(M), get_sequence_state_cost(As,M,[I],0,Ls,TC),myreverse(Ls,P),path_final_state(P). 

get_sequence_state_cost([],_,Ls,Lc,Ls,Lc).
get_sequence_state_cost([A|As],M,[S|L],Lcacc,Ls,Lc) :- next_state(S,A,M,SP,MP,C), Lcacc2 is Lcacc + C, get_sequence_state_cost(As,MP,[SP,S|L],Lcacc2,Ls,Lc).



%Example : 

% play_sequence_action([2,1],[[-1,0],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[1,0]],P,TC).

/* 
      Implementation of A*. Strategy : Start with a node 
      
*/

% Manhattan distance, i.e., |X2 - X1| + |Y2 - Y1|. Final state hard coded.
% heuristic(+S, -H, +TH) : H is Manhattan Distance between S and final state if TH = 1 otherwise H is 0
m_distance([X,Y],[V,Z],M) :- A is abs(V - X), B is abs(Z - Y), M is A+B. 

min_m_distance(S,L,A) :- min_m_distance(S, L, 100000, A). 

min_m_distance(_,[],M,M).
min_m_distance(S,[S2|L],A,M) :- m_distance(S,S2,C), (C < A -> min_m_distance(S,L,C,M); min_m_distance(S,L,A,M)).


heuristic(_,_,0,0).
heuristic(S,LSF,1,H) :- min_m_distance(S,LSF,H).


% Node expansion
% expand(+N, +As, +LN2,-LN) : LN is the concatenation of list of nodes expanded from N with actions As and LN2.
expand(_,[],LN,LN).
expand([X,M,La,P,C],[A|As],LN2,LN) :- next_state(X,A,M,XP,MP,C3) -> C2 is C3+C, expand([X,M,La,P,C],As,[[XP,MP,[A|La],[X|P],C2]|LN2],LN) ; expand([X,M,La,P,C],As,LN2,LN).


% greedy_get_node(+LN, +HC, +N2, -LN2, -N) - N is N2 or the least cost node of value HC in LN, LN2 is LN without the least cost node of value HC.  
greedy_get_node([],_,_,_,N,_,N).
greedy_get_node([NC|LN],LSF,HN,H2,N2,[N3|LN2],N) :- NC = [X,_,_,_,C], heuristic(X,LSF,HN,H), HC is H+C, (HC < H2 -> N3 = N2,greedy_get_node(LN,LSF,HN,HC,NC,LN2,N);N3 = NC,greedy_get_node(LN,LSF,HN,H2,N2,LN2,N)) .




multistage_astar(X,[W],M,As,H,PAcc,[P2|PAcc],C2) :- astar(X,[W],M,As,H,P3,W,_,C2), myreverse(P3,P2).
multistage_astar(X,[W,W2|LSF],M,As,H,PAcc,P,C) :- astar(X,[W,W2|LSF],M,As,H,P3,XF,MF,C2), myreverse(P3,P2), remove(XF,[W,W2|LSF],LSF2), multistage_astar(XF,LSF2,MF,As,H,[P2|PAcc],P,C4), C is C2+C4.

astar(M,As,H,P,C) :- coords_symbol(M,'G',LSF), coords_symbol(M,'.',[X|_]), multistage_astar(X,LSF,M,As,H,[],P2,C), myreverse(P2,P3), flatten_states(P3,P).
 

% astar(+X, +M, +As, +H, -P, -C) : P is the optimal path from X to the final state using the components of the search problem described above, given that the used heuristic H is admissible (i.e. between h0 and h*). C is the optimal cost.
astar(X, LSF, M, As, H, P, XF, MF, C) :- astar_aux([[X,M,[],[],0]], LSF,As, H, [XF,MF,_,P,C]).

% astar(+LN, +As, +H, -NF) - NF is the final node contained in LN. If LN does not contains a final node, the least sum heuristic (H) +cost node is expanded. Fail if there is no final or expandable node in LN.
astar_aux([N2 | LN4],LSF,As, HN,NF) :- N2 = [X,_,_,_,C],heuristic(X,LSF,HN,H), HC is H+C,greedy_get_node(LN4,LSF,HN,HC,N2,LN2,[X2|Xs]), (member(X2,LSF) -> NF = [X2|Xs] ; expand([X2|Xs],As,LN2,LN3), astar_aux(LN3,LSF,As,HN,NF)), !.

/*
	Execution examples
	
	maze(M,"maze.txt"), astar([2,1],M,[[0,1],[1,0],[-1,0],[0,-1]], P, C).

*/
%Example : 
%  
