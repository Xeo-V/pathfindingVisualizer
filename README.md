# pathfindingVisualizer
[//////////////------] 70%

## Table of Contents
- [Introduction](#introduction)
- [Description](#description)
- [Installation](#installation)
- [Usage](#usage)
- [How It Works](#how-it-works)
- [Features](#features)
- [Customization](#customization)
- [License](#license)
- [Future Goals](#future-goals)
- [Algorithms Used](#algorithms-used)

## Introduction
Pathfinding Visualizer is an interactive and user-friendly application designed to visualize various pathfinding algorithms. It helps users understand the mechanics behind the algorithms, observe their execution in real-time, and explore different scenarios by interacting with the grid.

## Description
The application offers a grid where the user can set the start and end points, draw walls, and visualize the pathfinding process. It supports multiple algorithms such as Dijkstra, A*, BFS, DFS, Swarm Algorithm, Greedy Best-First Search, and Bidirectional Search. The visualizer is built using Python and Tkinter, providing a seamless and responsive user experience.

## Installation
### Downloading the Project
1. Navigate to the project's [GitHub page](https://github.com/Xeo-V/pathfindingVisualizer).
2. Click on the "Code" button.
3. Download the ZIP file and extract it to your preferred location on your computer.

### Setting Up
1. Ensure you have Python installed on your computer. If not, download and install it from the [official website](https://www.python.org/downloads/).
2. Navigate to the project directory in your terminal or command prompt, type python3 to check if its installed correctly.
3. Extract the Zip file and you are good to go.

## Usage
1. Run the application with the command `python core.py`.
2. Use the control panel at the top to select the desired algorithm, set the start and end points, and visualize the pathfinding process.
3. Interact with the grid to draw walls and explore different scenarios.
4. Use the "Reset" button to clear the grid and start anew.

## How It Works
- **Set Start/End**: Use the respective buttons to set the start and end points on the grid.
- **Select Algorithm**: Choose the pathfinding algorithm from the dropdown menu.
- **Visualize**: Click the "Visualize" button to start the visualization.
- **Reset**: Clears the grid and all settings, allowing you to start over.

## Features
- Interactive Grid: Draw walls, set start and end points, and observe the pathfinding in real-time.
- Multiple Algorithms: Supports a variety of pathfinding algorithms.
- User-Friendly: Intuitive controls and responsive interface.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details. Anyone is free to use, change, and distribute the software.

## Customization
Users can modify the grid size and visualization speed by adjusting the respective variables in the `core.py` file. Feel free to explore and tailor the application to your preferences!

## Algorithms Used
Detailed explanations of the algorithms used in this project can be found in the [EXPLANATION.md](EXPLANATION.md) file. The following algorithms are implemented:
- Dijkstra
- A* (A Star)
- Breadth-First Search (BFS)
- Depth-First Search (DFS)
- Swarm Algorithm
- Greedy Best-First Search
- Bidirectional Search

## Future Goals
- Implement additional pathfinding algorithms.
- Enhance the user interface and add more customization options.
- Optimize performance for larger grids.
