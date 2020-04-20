p-Robust CBS algorighm 
----------------------


------  Important classes and methods ------

-- Program class  (the main class of the project)
In the Main method we choose which experiments to run (DragonAge/Grids/Maze/Specific Instance) from the folder "/bin/Debug(or Release)/instances/", if the folder doesn't contains an instance, a new instance will be created.

-- Run class (the class that runs the experiments and generates new instances)
In the Run method (constructor) we choose which solver to use (optimal/greedy p-Robust CBS with Deterministic/Monte-Carlo verifier).

-- ProblemInstance class (the class that holds the information about the instance)
In the Import method we get an instance name, read the corrisponding file of the instance, and convert it into a problem instance data structure.

-- CBS classes (the classes of the p-Robust algorithm)
In the Setup and Solve classes we setup the root of the constraint tree and solve the problem by performing a BFS on the tree.

-- CbsNode class (the class of a CBS node)
In the Solve method we find an optimal plan for each agent and identify the conflicts.
In the Replan method we replan for the constraint agent .

-- ClassicAStar class (the class of the A* algorithm)
In the solve method we find an optimal path for an agent with respect to a given set of constraints.



------  Program running process ------

1. Program.Main creates a new experiment and calls each instance with ProblemInstance.Import

2. The imported instance is given to Run.SolveGivenProblem, which calls Run.run

3. Run.run calls CBS.Setup in order to intialize the CBS root (CbsNode)

4. CbsNode.Solve finds a path for each agent with ClassicAStar.Solve and identifies the conflicts

5. Run.run calls CBS.Solve to perform a BFS on the constraint tree, starting from the root

6. Run.SolveGivenProblem prints the solution, if it is found