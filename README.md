astar
=====

Erlang Astar search algorithm library based on mwri

Making use of user functions to supply neighbour state and heuristic scores

mwri's version uses ets for {OpenQueue, ClosedSet} data storage, which is not easy to apply to the project, so it is modified

This version uses gb_trees for {OpenQueue, ClosedSet} data storage, process isolation

The following is quoted from the example of mwri:

Here's a worked example. The objective is to find the most
optimal route from 1 to 10. Paths is the function defining
each neighbour for each state. Scores returns the heuristic
score (in this case the closer the state is to 10 the higher
the score but also the longer the path followed so far the
lower the score).

1> Paths = fun  
    (1)  -> [ {2,2}, {5,5}        ];  
    (2)  -> [ {3,3}, {4,4}, {5,5} ];  
    (3)  -> [ {9,9}, {8,8}        ];  
    (4)  -> [ {5,5}, {1,1}        ];  
    (5)  -> [ {7,7}, {9,9}        ];  
    (9)  -> [ {10,10}, {1,1}      ];  
    (St) -> [ {1,1}               ]  
     end.  
#Fun<erl_eval.6.82930912>  
2> Scores = fun  
    (_, 10) -> max;  
    (PathSoFar, State) -> -length(PathSoFar)+State  
    end.  
#Fun<erl_eval.12.82930912>  
3> astar:search(1, Paths, Scores, []).  
{max,[10,9,5],10}  
4>  

The resultant tuple has 3 elements. The first element is the
best score, the second is the optimal path found, or partial
path, and the third is the final state reached.

For clear and more useful looking examples, see the unit tests.
This is one of the unit tests, it requires an optimal path to
be found from S to E (start to end), avoiding all the X's.

XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX  
X S  XXX                                                     X  
X    X                                                       X  
X XXXX                                                       X  
X                                                            X  
X                                                            X  
X              XXXX                                          X  
X            XXXXXXXXX                                       X  
X         XXXXXXXX                                           X  
X      XXXXXXXX                           XX                 X  
X                                         XX                 X  
X                     X                   XX                 X  
X                    XXX                  XX                 X  
X XXXXXXXXXXXXX      XXX                  XX                 X  
X    XXXXXX           X                   XX                 X  
X      XX                                 XX                 X  
X                                         XX                 X  
X                                         XX                 X  
X                 XXXXXX                  XX                 X  
X                XXXXXXXXXXXX             XXX                X  
X                    XXXXXXX         XXXXXXXX                X  
X                                  XXXXXXXXXXX               X  
X                                 XXXXXXXXXXXXXX             X  
X                               XXXXXXXXXX                   X  
X                       XXXXXXXXXXXXXXXXXXX                  X  
X                               XXXXXXXXXXXXXX               X  
X                                          XXXXXXXX          X  
X                                                            X  
X                                                            X  
X                                                            X  
X                                         X                  X  
X                                         X                  X  
X                                        XX                  X  
X         XXX                         XXXX                   X  
X       XXXXXXXX                XXXXXXXX                     X  
X     XXXXXXXXXXX      XXXXXXXXXXXXXX                        X  
X  XXXXXXXXXXXXXXXXXXXXXXXXX                                 X  
X      XXXXXXXXXXXXXXX                                       X  
X                                                       X    X  
X                                                      XX    X  
X                                                    XXX     X  
X               XXXXXX                           XXXXXXX     X  
X                 XXX                XXXXXXXXXXXXXXXXX       X  
X                               XXXXXXXXXXXXXXXXXXX          X  
X                                                            X  
X                                                            X  
X                     XXXXXXXXXXXXXXX                        X  
X                 XXXXXXX        XXXX                        X  
X             XXXXXXX              XX                        X  
X         XXXXXXXXXXXXXX            X            XXXXXXXXXXXXX  
X           XXXXXXXXXXXXXX                                   X  
X                                                            X  
X                                       XXXXXXXXXXXXX        X  
X                                      XXXXXXXXXXXXXXXXXXXX  X  
X                                     XXXXXXXXXXXXXXX        X  
X                                    XXXXXXXXXXXX            X  
X                                           XXX              X  
X                                            X               X  
X                                                          E X  
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX  

The result currently found by the algorithm is as follows:

XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX  
X S  XXX                                                     X  
Xoo  X                                                       X  
XoXXXX                                                       X  
X o                                                          X  
X  o                                                         X  
X   o          XXXX                                          X  
X    o       XXXXXXXXX                                       X  
X     o   XXXXXXXX                                           X  
X     oXXXXXXXX                           XX                 X  
X      o                                  XX                 X  
X       o             X                   XX                 X  
X        oooooo      XXX                  XX                 X  
X XXXXXXXXXXXXXo     XXX                  XX                 X  
X    XXXXXX     o     X                   XX                 X  
X      XX       o                         XX                 X  
X               o                         XX                 X  
X               o                         XX                 X  
X               o XXXXXX                  XX                 X  
X               oXXXXXXXXXXXX             XXX                X  
X                o   XXXXXXX         XXXXXXXX                X  
X                 o                XXXXXXXXXXX               X  
X                  o              XXXXXXXXXXXXXX             X  
X                   o           XXXXXXXXXX                   X  
X                    o  XXXXXXXXXXXXXXXXXXX                  X  
X                     o         XXXXXXXXXXXXXX               X  
X                      o                   XXXXXXXX          X  
X                       o                                    X  
X                        o                                   X  
X                         ooooooooooooooooo                  X  
X                                         Xo                 X  
X                                         X o                X  
X                                        XX  o               X  
X         XXX                         XXXX    o              X  
X       XXXXXXXX                XXXXXXXX       o             X  
X     XXXXXXXXXXX      XXXXXXXXXXXXXX           o            X  
X  XXXXXXXXXXXXXXXXXXXXXXXXX                     o           X  
X      XXXXXXXXXXXXXXX                            ooooooo    X  
X                                                       Xo   X  
X                                                      XXo   X  
X                                                    XXXo    X  
X               XXXXXX                           XXXXXXXo    X  
X                 XXX                XXXXXXXXXXXXXXXXX o     X  
X                               XXXXXXXXXXXXXXXXXXX   o      X  
X                                                    o       X  
X                                                   o        X  
X                     XXXXXXXXXXXXXXX              o         X  
X                 XXXXXXX        XXXX             o          X  
X             XXXXXXX              XX            o           X  
X         XXXXXXXXXXXXXX            X           oXXXXXXXXXXXXX  
X           XXXXXXXXXXXXXX                       o           X  
X                                                 ooo        X  
X                                       XXXXXXXXXXXXXoooooo  X  
X                                      XXXXXXXXXXXXXXXXXXXXo X  
X                                     XXXXXXXXXXXXXXX      o X  
X                                    XXXXXXXXXXXX          o X  
X                                           XXX            o X  
X                                            X             o X  
X                                                          E X  
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX  



Build
-----

    $ rebar3 compile
