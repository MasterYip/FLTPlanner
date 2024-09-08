from ompl import base
from ompl import geometric
import matplotlib.pyplot as plt

from fast_legged_planner_py.utils.benchmark.benchmark import do_cprofile


def isStateValid(state):
    # Define the bounds for the state space
    inrange = (state[0] > 0 and state[0] < 5 and state[1] > 0 and state[1] < 5)
    obs = (state[0] - 2.5)**2 + (state[1] - 2.5)**2 > 1.0
    return obs and inrange


@do_cprofile(save=True, sortby="tottime")
def plan(maxtime=0.01, type="rrt_star"):
    """
    :param type: rrt, informed_rrt_star, rrt_star
    """
    # Create an instance of the state space
    space = base.RealVectorStateSpace(2)

    # Set bounds for the state space
    bounds = base.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(5)
    space.setBounds(bounds)

    # Create an instance of the space information
    si = base.SpaceInformation(space)

    # Set the state validity checker
    si.setStateValidityChecker(base.StateValidityCheckerFn(isStateValid))

    # Create an instance of the problem definition
    pdef = base.ProblemDefinition(si)

    # Set the start and goal states
    start = base.State(space)
    start[0] = 1.0
    start[1] = 1.0
    goal = base.State(space)
    goal[0] = 4.0
    goal[1] = 4.0
    pdef.setStartAndGoalStates(start, goal)

    if type == "rrt":
        # Create an instance of the planner (RRT)
        planner = geometric.RRT(si)
    elif type == "informed_rrt_star":
        # Create an instance of the planner (InformedRRTstar)
        planner = geometric.InformedRRTstar(si)
    elif type == "rrt_star":
        # Create an instance of the planner (RRTstar)
        planner = geometric.RRTstar(si)

    # FIXME
    # if type == "informed_rrt_star":
    #     # Set the optimization objective for the planner
    #     objective = base.PathLengthOptimizationObjective(si)
    #     planner.setOptimizationObjective(objective)

    # Set the problem definition for the planner
    planner.setProblemDefinition(pdef)

    # Specify the maximum time to perform planning
    planner.setup()
    solved = planner.solve(maxtime)

    # If a solution is found, retrieve the solution path
    if solved:
        path = pdef.getSolutionPath()
        path.interpolate(100)  # Interpolate path for smoother visualization
        return path

    return None


def plot_path(path):
    # Extract states from the solution path
    states = path.getStates()
    x = [state[0] for state in states]
    y = [state[1] for state in states]

    # Plot the environment
    plt.plot([0, 5, 5, 0, 0], [0, 0, 5, 5, 0], 'k-', linewidth=2)

    # Plot the solution path
    plt.plot(x, y, 'r-', linewidth=2)

    # Set plot limits and labels
    plt.xlim(0, 5)
    plt.ylim(0, 5)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')

    # Show the plot
    plt.show()


if __name__ == "__main__":
    # Plan the path
    solution_path = plan()

    if solution_path:
        # Visualize the solution path
        plot_path(solution_path)
    else:
        print("No solution found.")
