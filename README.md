# D* Path Planning Algorithm

### D* Lite Pathfinding Visualization
This project implements a visualization of the D* Lite pathfinding algorithm on a 2D grid map using Python. D* Lite is designed for efficient path planning in dynamic environments where obstacles may appear or disappear over time. Unlike traditional A*, which recomputes paths from scratch, D* Lite incrementally updates its existing path, making it ideal for real-time robotic navigation.

The project features:
- Interactive 2D grid map with dynamic obstacle updates
- Visual explanation of node expansion and path repair
- Clean Python implementation with clear data structures

This tool is useful for students and researchers learning about adaptive pathfinding, autonomous agents, and search-based planning.

## DEMO
![螢幕錄影 Jun 17 2025 (1)](https://github.com/user-attachments/assets/a5c7630c-4e00-4258-b2c9-05d140d0ed41)

### Installation
clone the repository
```bash
 
git clone https://github.com/EricChen0104/D_star_lite_Algorithm_PYTHON.git
cd D_star_lite_Algorithm_PYTHON
 
```
install the requirements
```bash
 
pip install -r "requirements.txt"
 
```

## D* Lite Algorithm Introduction

D* Lite is an incremental heuristic search algorithm designed for path planning in dynamic environments. It can be seen as an optimized extension of A*, capable of efficiently updating paths when the map changes (e.g., newly discovered obstacles), rather than recomputing from scratch like standard A*.

### Basic Concepts
Each node maintains two key values:

- `g(n)`: the current known cost from the start node to node `n`
- `rhs(n)`: one-step lookahead value, representing the best cost to reach `n` through any predecessor

The node priority is determined by a key function (similar to A*):

`key(n) = [min(g(n), rhs(n)) + h(start, n), min(g(n), rhs(n))]`

The algorithm always expands the node with the **smallest key**, and incrementally repairs the path when changes occur in the map.

### Heuristic Function h(n)
As with A*, a popular heuristic is the **Manhattan Distance**:

`h(n) = |Xstart - Xn| + |Ystart - Yn|`

This allows D* Lite to maintain efficiency while guaranteeing optimality, even as obstacles change.

## References
- Stentz, A. (1994). The D* algorithm for real-time planning of optimal traverses (p. 34). Carnegie Mellon University, the Robotics Institute.
- https://www.cs.cmu.edu/~motionplanning/lecture/AppH-astar-dstar_howie.pdf 

