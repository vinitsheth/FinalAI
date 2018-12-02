# lifelong-planning-pacman

Environment
Run on python 2.7

Implement D* Lite for robot path finding.
Implemented Lifelong Planing A*

Run commands

For dstar and size2020Maze
python pacman.py -l size2020Maze -z .5 -p SearchAgent -a fn=dstar,heuristic=manhattanHeuristic

For dstar and MediumMaze
python pacman.py -l MediumMaze -z .5 -p SearchAgent -a fn=dstar,heuristic=manhattanHeuristic

For dstar and BigMaze
python pacman.py -l BigMaze -z .5 -p SearchAgent -a fn=dstar,heuristic=manhattanHeuristic


For astar and size2020Maze
python pacman.py -l size2020Maze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic

For astar and MediumMaze
python pacman.py -l MediumMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic

For astar and BigMaze
python pacman.py -l BigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic


For life long planing a* and size2020Maze
python pacman.py -l size2020Maze -z .5 -p SearchAgent -a fn=lpastar,heuristic=manhattanHeuristic

For life long planing a* and MediumMaze
python pacman.py -l MediumMaze -z .5 -p SearchAgent -a fn=lpastar,heuristic=manhattanHeuristic

For life long planing a* and BigMaze
python pacman.py -l BigMaze -z .5 -p SearchAgent -a fn=lpastar,heuristic=manhattanHeuristic

If you wish to see all the walls in the layout from starting then uncomment line 210 in graphicsDisplay.py

If you wish to add more layouts, create a layout as shown in layouts folder and save it as .lay extention. There should only be one dot(.) representating food (goalState) and only one P (representating Pacman start position) and rest should be % representating wall


