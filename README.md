astar
=====

Erlang A* search algorithm library based on mwri

Making use of user functions to supply neighbour state and heuristic scores

mwri's version uses ets for {OpenQueue, ClosedSet} data storage, which is not easy to apply to the project, so it is modified

This version uses gb_trees for {OpenQueue, ClosedSet} data storage, process isolation

You can check aster_tests.erl to comfirm the eunit/example


Build
-----

    $ rebar3 compile
