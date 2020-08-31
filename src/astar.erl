%% @author WangLiangyou
%% @copyright 2018 WangLiangyou
%% 
%% This file is part of the Erlang A* search library called 'astar'.
%% 
%% 'astar' is free software: you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation, either version 3 of the License, or
%% (at your option) any later version.
%% 
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%% 
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%% 
%% @doc Main API module for 'astar' app.


-module(astar).


-export([search/4]).


-include_lib("eunit/include/eunit.hrl").


-define(WORK_LIMIT, 10000).


%% @doc Run an A* search.
%%
%% Run an A* search, starting from the initial state (the first
%% parameter) and working until exhaustion (the work limit is
%% reached) or the maximum score is achieved.
%%
%% In order to work, the search algorithm must call a user
%% supplied function which returns the neighbouring states of
%% any given state (the second parameter) and another user
%% supplied function that returns a heuristic score for each
%% state (the third parameter).
%%
%% The neighbour function must in fact return a set of 2 element
%% tuples, the first being the path from the current state to
%% the neighbouring state, and the second being the neighbouring
%% state. So if states 4, 5 and 8 can be reached from state 2
%% then NeighbourFn(2) must return something like [{2,4}, {3,5},
%% {6,8}]. In this case the first elements are numbers added to
%% the current state (2) to get to the new (neighbouring) states
%% (4, 5 and 8), but they could be anything really that identifies
%% that path in that direction between the two states. So for
%% example [{4,4}, {5,5}, {8,8}] or [{1,4}, {2,5}, {3,8}] would
%% both be just as valid, but [{3,4}, {2,5}, {3,8}] would not
%% because the 3 is an ambiguous route identifier.
%%
%% The score function must return an integer heuristic score
%% indicating the proximity to the goal state (lower is closer),
%% or the atom 'max' (which means the goal state has been reached
%% and no further searching is useful).
%% 
%% With these functions supplied, the search algorithm can find
%% all possible reachable states, eventually, and return an optimal
%% route (the element 1 tuples).
%%
%% There is also a fourth parameter, which is a list of options.
%%
%% The only supported option is currently 'worklimit', which
%% is 10,000 by default. If you include the tuple {worklimit, N}
%% in options then this will override the default and more, or
%% less work will be done before giving up.
%%
%% Here's a worked example. The objective is to find the most
%% optimal route from 1 to 10. Paths is the function defining
%% each neighbour for each state. Scores returns the heuristic
%% score (in this case the closer the state is to 10 the lower
%% the score but also the longer the path followed so far the
%% higher the score).
%% 
%% <pre>
%% 1> Paths = fun
%%     (1)  -> [ {2,2}, {5,5}        ];
%%     (2)  -> [ {3,3}, {4,4}, {5,5} ];
%%     (3)  -> [ {9,9}, {8,8}        ];
%%     (4)  -> [ {5,5}, {1,1}        ];
%%     (5)  -> [ {7,7}, {9,9}        ];
%%     (9)  -> [ {10,10}, {1,1}      ];
%%     (St) -> [ {1,1}               ]
%%      end.
%% #Fun<erl_eval.6.82930912>
%% 2> Scores = fun
%%     (_, 10) -> max;
%%     (PathSoFar, State) -> -length(PathSoFar)+State
%%     end.
%% #Fun<erl_eval.12.82930912>
%% 3> astar:search(1, Paths, Scores, []).
%% {max,[10,9,5],10}
%% 4> 
%% </pre>
%%
%% The resultant tuple has 3 elements. The first element is the
%% best score, the second is the optimal path found, or partial
%% path, and the third is the final state reached.
%% 
%% The real validity of this example is a little doubtful as the
%% balance of path length and numerical proximity to 10 might not
%% be sensible, but heuristics aren't supposed to be perfect.
%% The unit tests provide clearly sensible examples with graphic
%% illustration of the optimal results found.

search(Start, NeighbourFn, ScoreFn, Options) ->
	{OpenTree0, ClosedList} = data_sets(),
	OpenTree = push(OpenTree0, [], Start, ScoreFn([], Start)),
	WorkLimit =
		case lists:keysearch(worklimit, 1, Options) of
			false -> ?WORK_LIMIT;
			{value, {worklimit, A}} when is_integer(A) -> A
		end,
	continue(OpenTree, ClosedList, NeighbourFn, ScoreFn, WorkLimit).


continue(OpenTree, ClosedList, _NeighbourFn, _ScoreFn, Count)
  when Count == 0 ->
	{{BestScore, Path, State}, _LeftOpenTree, _NewClosedList} = pop_best(OpenTree, ClosedList),
	{{worklimited, BestScore}, Path, State};
continue(OpenTree, ClosedList, NeighbourFn, ScoreFn, Count) ->
	case pop_best(OpenTree, ClosedList) of
		none ->
			none;
		{{max, Path, State}, _, _} ->
			{max, Path, State};
		{{_BestScore, Path, State}, LeftOpenTree, NewClosedList} ->
			List =
				lists:filter(fun({_Npath, Nstate}) ->
									 not is_closed(NewClosedList, Nstate) andalso not is_open(LeftOpenTree, Nstate)
							 end, NeighbourFn(State)),
			NewOpenTree =
				lists:foldl(fun({Npath, Nstate}, AccOpenTree) ->
									Nfullpath = [Npath|Path],
									Res = push(AccOpenTree, Nfullpath, Nstate, ScoreFn(Nfullpath, Nstate)),
									Res
							end, LeftOpenTree, List),
			continue(NewOpenTree, NewClosedList, NeighbourFn, ScoreFn, Count - 1)
	end.


data_sets() ->
	OpenTree = gb_trees:empty(),
	ClosedList = [],
	{OpenTree, ClosedList}.


push(OpenTree, Path, State, Score) ->
	case gb_trees:is_defined(Score, OpenTree) of
		false -> gb_trees:insert(Score, {Path, State}, OpenTree);
		true -> gb_trees:update(Score, {Path, State}, OpenTree)
	end.


pop_best(OpenTree, ClosedList) ->
	case gb_trees:is_empty(OpenTree) of
		true -> none;
		false ->
			{BestScore, {Path, State}, LeftOpenTree} = gb_trees:take_largest(OpenTree),
			NewClosedList = [State|ClosedList],
			Best = {BestScore, Path, State},
			{Best, LeftOpenTree, NewClosedList}
	end.

is_open(OpenTree, State) ->
	gb_trees:is_defined(State, OpenTree).

is_closed(ClosedList, State) ->
	lists:member(State, ClosedList).

