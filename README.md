# mapf-cbs-heuristics-public
**MAPF: Implementation of CBS with Heuristics**

This is a group project for a 4th year level course in Intelligent Systems.
_Note: Some code may be redacted._

<h2>Abstract</h2>

**Conflict-Based Search** **(CBS)** is a well known path finding algorithm that has proved to provide effective results. Current research works has provided several methods to improve its performance, including admissible heuristics. In this paper, our focus is on cardinal, semi and non-cardinal conflicts. This involves implementing **Conflict Graph (CG)**, **Dependency Graph (DG)** and **Weighted Dependency Graph (WDG**) heuristics. By testing against 8x8 maps, our results will show an increase in run-time and percentage of solved instances, and a decrease in number of nodes expanded.

<h2>Implementation</h2>
CBS with heuristics begins with computing all possible shortest paths for every pair of agents whose paths in _N.solution_ contain no collisions. To do this we perform a breadth first search, with a bound on cost. We prune nodes which do not satisfy constraints and nodes whose cost plus remaining distance do not satisfy the cost bound. Because the maps contain many overlapping paths, we did not use a closed list in our breadth first search. Rather, we relied on the constraints to prune the extra nodes and ran the search until there were no nodes left in the open list.
To reduce the amount of times the breadth first search would need to be performed, we implemented a list which stores the previous iteration’s minimal cost path list. Before doing the search, this list is checked. If the current cost for that agent is the same as the previous one, then we just check the previous list against the new constraints and use that. Doing this made the program significantly faster, as it largely reduced the number of times the breadth first search had to be performed.
Then, construct a Multi-Valued Decision Di- agram using those paths and compute all cardinal, semi and non-cardinal conflicts. These conflicts are defined by:
1. _cardinal conflict_: occurs when all shortest paths of two agents contain a conflict
2. _semi-cardinal conflict_: occurs when one all shortest paths of one agent has a conflict with just one shortest path of another agent
3. _non-cardinal conflict_: occurs when both agents have multiple shortest paths available to them and a single conflict exists in their respective paths.

<h2>Experimental Results</h2>

<p float="left">
<img src="https://github.com/nour-habib/mapf-cbs-heuristics-public/blob/main/fig1.png" width="450">
<img src="https://github.com/nour-habib/mapf-cbs-heuristics-public/blob/main/fig2.png" width="450">
<img src="https://github.com/nour-habib/mapf-cbs-heuristics-public/blob/main/fig3.png" width="450">
<img src="https://github.com/nour-habib/mapf-cbs-heuristics-public/blob/main/fig4.png" width="450">
<img src="https://github.com/nour-habib/mapf-cbs-heuristics-public/blob/main/fig5.png" width="450">
<img src="https://github.com/nour-habib/mapf-cbs-heuristics-public/blob/main/fig6.png" width="450">
<img src="https://github.com/nour-habib/mapf-cbs-heuristics-public/blob/main/average-hvalue.png" width="450">
</p>
