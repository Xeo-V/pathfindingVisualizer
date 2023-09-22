# core.py

import tkinter as tk
from tkinter import ttk
from working import dijkstra, a_star, bfs, dfs

class PathfindingVisualizer:
    def __init__(self, master):
        self.master = master
        self.master.title("Pathfinding Visualizer")
        self.algorithm = tk.StringVar(value="Dijkstra")  # Default algorithm
        self.rows = 20  # Number of rows in the grid
        self.cols = 20  # Number of columns in the grid
        self.cell_size = 25  # Size of each cell in pixels
        self.start = None  # Start point coordinates
        self.end = None  # End point coordinates
        self.mode = None  # Mode can be 'start', 'end', 'wall', or None
        self.create_widgets()
        
    def create_widgets(self):
        # Create dropdown to select the algorithm
        algo_label = tk.Label(self.master, text="Algorithm:")
        algo_label.pack(side="left")
        
        algo_dropdown = ttk.Combobox(self.master, textvariable=self.algorithm, values=["Dijkstra", "A*", "BFS", "DFS"])
        algo_dropdown.pack(side="left")
        
        # Create buttons to set start, set end, visualize, and reset
        start_button = tk.Button(self.master, text="Set Start", command=self.set_start)
        start_button.pack(side="left")
        
        end_button = tk.Button(self.master, text="Set End", command=self.set_end)
        end_button.pack(side="left")
        
        visualize_button = tk.Button(self.master, text="Visualize", command=self.visualize)
        visualize_button.pack(side="left")
        
        reset_button = tk.Button(self.master, text="Reset", command=self.reset)
        reset_button.pack(side="left")
        
        # Create the grid
        self.create_grid()
        
    def create_grid(self):
        self.grid_frame = tk.Frame(self.master)
        self.grid_frame.pack(side="bottom")
        
        self.grid = []
        for row in range(self.rows):
            row_cells = []
            for col in range(self.cols):
                cell = tk.Button(self.grid_frame, width=self.cell_size, height=self.cell_size, bg="white",
                                 command=lambda r=row, c=col: self.on_cell_clicked(r, c))
                cell.grid(row=row, column=col, sticky="nsew")
                row_cells.append(cell)
            self.grid.append(row_cells)
    
    def on_cell_clicked(self, row, col):
        if self.mode == 'start':
            if self.start is not None:
                self.grid[self.start[0]][self.start[1]].config(bg="white")
            self.grid[row][col].config(bg="green")
            self.start = (row, col)
        elif self.mode == 'end':
            if self.end is not None:
                self.grid[self.end[0]][self.end[1]].config(bg="white")
            self.grid[row][col].config(bg="red")
            self.end = (row, col)
        elif self.mode == 'wall':
            cell_color = self.grid[row][col].cget("bg")
            new_color = "black" if cell_color == "white" else "white"
            self.grid[row][col].config(bg=new_color)
        self.master.config(cursor='arrow')
        self.mode = None
        
    def set_start(self):
        self.mode = 'start'
        self.master.config(cursor='cross')
        
    def set_end(self):
        self.mode = 'end'
        self.master.config(cursor='cross')
        
    def visualize(self):
        # Check if start and end points are set
        if self.start is None or self.end is None:
            tk.messagebox.showwarning("Warning", "Please set both start and end points.")
            return
    
        # Create a grid representation with True for walls and False for open spaces
        grid = [[self.grid[row][col].cget("bg") == "black" for col in range(self.cols)] for row in range(self.rows)]
    
        # Get the selected algorithm from the dropdown
        algorithm = self.algorithm.get()
    
        # Call the corresponding pathfinding algorithm from working.py
        if algorithm == "Dijkstra":
            path = dijkstra(grid, self.start, self.end)
        elif algorithm == "A*":
            path = a_star(grid, self.start, self.end)
        elif algorithm == "BFS":
            path = bfs(grid, self.start, self.end)
        elif algorithm == "DFS":
            path = dfs(grid, self.start, self.end)
        else:
            tk.messagebox.showerror("Error", f"Unknown algorithm {algorithm}")
            return
    
        # Visualize the resulting path
        if not path:
            tk.messagebox.showinfo("Info", "No path found.")
            return
    
        for row, col in path:
            self.grid[row][col].config(bg="blue")
            self.master.update_idletasks()
            self.master.after(50)  # Add a delay to visualize the path being constructed

        
    def reset(self):
        # Reset the start and end points
        self.start = None
        self.end = None
        self.mode = None
        self.master.config(cursor='arrow')

        # Clear the grid
        for row in range(self.rows):
            for col in range(self.cols):
                self.grid[row][col].config(bg="white")


def main():
    root = tk.Tk()
    app = PathfindingVisualizer(master=root)
    root.mainloop()


if __name__ == "__main__":
    main()
