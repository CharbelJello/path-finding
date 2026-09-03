# Grid Pathfinding: BFS vs A*

A small C++17 exercise comparing breadth-first search and A* on a 2D grid with random obstacles.

I built this while preparing an application for an undergraduate research project on path planning in porous self-reconfigurable modular robots. My algorithms background is in Java, so this was mostly a way to move that into modern C++ and to get a concrete feel for how a heuristic changes a search.

## What it does

- Generates a 15x40 grid where each cell has a 25% chance of being a wall. The generator is seeded, so every run produces the same map.
- Finds a shortest path from the top-left corner to the bottom-right corner, first with BFS and then with A* using the Manhattan distance heuristic.
- Prints each path over the grid and reports how many nodes each search expanded before reaching the goal.

## Result on the default map

Both algorithms return a 55-step path. BFS expands 458 nodes; A* expands 320.

```
BFS traversal:
Nodes expanded (BFS): 458
S#...#..#.#..#...##..#.##.......#...###.
*.#.#..#.#.................##...##......
**#..........#.#.#.#....#..#......#.#...
#*...#..#....###..#.#..#....#.......#.#.
.*...#....#...#......#..##.......#......
.************##.#.................#...#.
.........#.#*.....##......#......##.##.#
.#....#....#*.......#..##..#....##..#..#
##......#.##*##..##.....#...#...........
.#.....#.#.#***********#***....#....#..#
.....##.......##..#..#***#*********#..#.
#...#.......#..........#..#......#**..##
#.......#.#.#..#....#....##....#..#*....
....#####........##..#..#.#......#.*****
.......#...........##..#.......#......#G

Path length: 55 steps
```

```
A* traversal:
Nodes expanded (A*): 320
S#...#..#.#..#...##..#.##.......#...###.
**#.#..#.#.................##...##......
.*#..........#.#.#.#....#..#......#.#...
#****#..#....###..#.#..#....#.......#.#.
....*#....#...#......#..##.......#......
....*********##.#***************..#...#.
.........#.#******##......#....*.##.##.#
.#....#....#........#..##..#...*##..#..#
##......#.##.##..##.....#...#..*******..
.#.....#.#.#...........#.......#....#*.#
.....##.......##..#..#...#.........#.*#.
#...#.......#..........#..#......#...*##
#.......#.#.#..#....#....##....#..#..***
....#####........##..#..#.#......#.....*
.......#...........##..#.......#......#G

Path length: 55 steps
```

The two paths differ because there are many shortest paths on this grid and each search reaches a different one first. Both are optimal.

## Why A* expands fewer nodes

BFS explores outward in every direction equally, so it visits a lot of cells that lead away from the goal. A* orders its queue by `f = g + h`, where `g` is the steps taken so far and `h` is the Manhattan distance to the goal. Cells heading the wrong way get a high `f` and wait.

Manhattan distance never overestimates the true remaining cost on a 4-connected grid (you need at least that many moves, and walls only add more), so the heuristic is admissible and A* stays optimal while pruning.

## Implementation notes

- The grid is stored as a flat `std::vector<char>`; cell `(r, c)` lives at index `r * cols + c`. The same index doubles as a node ID for the `parent`, `seen`, `gScore`, and `closed` arrays.
- BFS marks cells as seen when they are pushed, not when they are popped, so no cell enters the queue twice.
- A* uses a `std::priority_queue` of `(f, id)` pairs with lazy deletion: a cell can be pushed more than once with different `f` values, and stale entries are skipped on pop via the `closed` array.
- Paths are rebuilt by walking the `parent` array backwards from the goal.

## Build and run

Visual Studio: open the solution, select x64, build and run.

Command line:

```
g++ -std=c++17 -O2 path-finding.cpp -o pathfinding
./pathfinding
```
