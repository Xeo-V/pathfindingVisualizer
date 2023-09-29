import tkinter as tk
from tkinter import ttk
from working import dijkstra, a_star, bfs, dfs, swarm, greedy_best_first, bidirectional_search, ant_colony_optimization, genetic_algorithm
import threading

class PathfindingVisualizer:
    def __init__(self, master):
        self.master = master
        self.master.title("Pathfinding Visualizer")
        self.algorithm = tk.StringVar(value="Dijkstra") 
        self.rows = 30  
        self.cols = 80  
        self.cell_size = 1
        self.start = None 
        self.end = None 
        self.mode = None 
        self.create_widgets()
        
    def create_widgets(self):
        control_frame = tk.Frame(self.master)
        control_frame.pack(side="top", fill="x")
        
        algo_label = tk.Label(control_frame, text="Algorithm:")
        algo_label.pack(side="left")
        
        algo_dropdown = ttk.Combobox(self.master, textvariable=self.algorithm, values=["Dijkstra", "A*", "BFS", "DFS", "Swarm", "Greedy Best-First", "Bidirectional", "Ant Colony Optimization", "Genetic Algorithm",])
        algo_dropdown.pack(side="left")
        
        start_button = tk.Button(control_frame, text="Set Start", command=self.set_start)
        start_button.pack(side="left")
        
        end_button = tk.Button(control_frame, text="Set End", command=self.set_end)
        end_button.pack(side="left")
        
        visualize_button = tk.Button(control_frame, text="Visualize", command=self.visualize)
        visualize_button.pack(side="left")
        
        reset_button = tk.Button(control_frame, text="Reset", command=self.reset)
        reset_button.pack(side="left")
        
        self.create_grid()
        
    def create_grid(self):
        self.grid_frame = tk.Frame(self.master)
        self.grid_frame.pack(side="top", fill="both", expand=True)
        
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
        threading.Thread(target=self.run_visualization).start()

    def run_visualization(self):
        try:
            if self.start is None or self.end is None:
                tk.messagebox.showwarning("Warning", "Please set both start and end points.")
                return
        
            grid = [[self.grid[row][col].cget("bg") == "black" for col in range(self.cols)] for row in range(self.rows)]
        
            algorithm = self.algorithm.get()
        
            if algorithm == "Dijkstra":
                path = dijkstra(grid, self.start, self.end)
            elif algorithm == "A*":
                path = a_star(grid, self.start, self.end)
            elif algorithm == "BFS":
                path = bfs(grid, self.start, self.end)
            elif algorithm == "DFS":
                path = dfs(grid, self.start, self.end)
            elif algorithm == "Swarm":
                path = swarm(grid, self.start, self.end)
            elif algorithm == "Greedy Best-First":
                path = greedy_best_first(grid, self.start, self.end)
            elif algorithm == "Bidirectional":
                path = bidirectional_search(grid, self.start, self.end)
            elif algorithm == "Ant Colony Optimization":
                path = ant_colony_optimization(grid, self.start, self.end, num_ants=10, num_iterations=100)
            elif algorithm == "Genetic Algorithm":
                path = genetic_algorithm(grid, self.start, self.end, population_size=10, num_generations=100)
            else:
                tk.messagebox.showerror("Error", f"Unknown algorithm {algorithm}")
                return
        
            if not path:
                tk.messagebox.showinfo("Info", "No path found.")
                return
        
            for row, col in path:
                self.master.after(0, lambda r=row, c=col: self.grid[r][c].config(bg="blue"))
                self.master.update_idletasks()
                self.master.after(50)
    
        except Exception as e:
            tk.messagebox.showerror("Error", f"An error occurred: {e}")
            
               
    def reset(self):
        self.start = None
        self.end = None
        self.mode = None
        self.master.config(cursor='arrow')

        for row in range(self.rows):
            for col in range(self.cols):
                self.grid[row][col].config(bg="white")


def main():
    root = tk.Tk()
    app = PathfindingVisualizer(master=root)
    root.mainloop()


if __name__ == "__main__":
    main()
