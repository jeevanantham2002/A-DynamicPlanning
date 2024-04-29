import hybrid_astar_wrapper
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch


def main():
    """A debug script for the hybrid astar planner.

    This script will solve the hybrid astar problem in a
    standalone simulation and visualize the results or raise an error if a
    path is not found.
    """
    print(__file__ + " start!!")
    sim_loop = 1000
    area = 20.0  # animation area length [m]
    show_animation = True
    obstacle_speed_per_iteration = .2

    obstacle_list = np.array([[26.0, 10.0, 34.0, 17.0]])
    number_of_obstacles = len(obstacle_list)
    true_start = np.array([10, 15, 0])
    true_end = np.array([50, 15, 0])

    start = np.array([10, 15, 0])
    end = np.array([50, 15, 0])

    start_time = time.time()
    best_path = []
    obstacle_path = []

    # Init initial positions for path
    best_path.append([start[0], start[1], start[2]])
    for obstacle in obstacle_list:
        obstacle_path.append([obstacle[0], obstacle[1], obstacle[2], obstacle[3]])

    while np.hypot(start[0] - end[0], start[1] - end[1]) > 2:
        initial_conditions = {
            'start': start,
            'end': end,
            'obs': obstacle_list,
        }

        hyperparameters = {
            "step_size": 1.0,
            "max_iterations": 10000,
            "completion_threshold": 1.0,
            "angle_completion_threshold": 3.0,
            "rad_step": 0.5,
            "rad_upper_range": 4.0,
            "rad_lower_range": 4.0,
            "obstacle_clearance": 0.5,
            "lane_width": 6.0,
            "radius": 6.0,
            "car_length": 4.8,
            "car_width": 1.8,
        }

        result_x, result_y, result_yaw, success = \
            hybrid_astar_wrapper.apply_hybrid_astar(initial_conditions,
                                                    hyperparameters)
        if not success:
            print("FAILED")
            ## Modify so that we try to take a path that is way ahead of our end goal then
            ## Todo
            continue

        print(best_path)

        if len(result_x) != 0 and len(result_y) != 0:
            best_path.append([result_x[1], result_y[1], result_yaw[1]])
            start = np.array([result_x[1], result_y[1], result_yaw[1]])

        for obstacle in obstacle_list:
            obstacle[0] = obstacle[0] + obstacle_speed_per_iteration
            obstacle[2] = obstacle[2] + obstacle_speed_per_iteration
            obstacle_path.append([obstacle[0], obstacle[1], obstacle[2], obstacle[3]])

    end_time = time.time() - start_time
    print("Time taken: {}s".format(end_time))
    print(best_path)

    for i in range(len(best_path)):
        time.sleep(.1)
        x = best_path[i][0]
        y = best_path[i][1]
        yaw = best_path[i][2]

        if np.hypot(x - end[0], y - end[1]) <= 2:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None]
            )
            ax = plt.gca()

            for j in range(number_of_obstacles):
                bottom_left_x = obstacle_path[i * number_of_obstacles + j][0]
                bottom_left_y = obstacle_path[i * number_of_obstacles + j][1]
                top_right_x = obstacle_path[i * number_of_obstacles + j][2]
                top_right_y = obstacle_path[i * number_of_obstacles + j][3]
                rect = patch.Rectangle((bottom_left_x, bottom_left_y),
                                       top_right_x - bottom_left_x,
                                       top_right_y - bottom_left_y)
                ax.add_patch(rect)

            plt.plot(start[0], start[1], "og")
            plt.plot(end[0], end[1], "or")

            plt.plot(x, y, "vc")
            plt.xlim(x - area, x + area)
            plt.ylim(y - area, y + area)

            plt.xlabel("X axis")
            plt.ylabel("Y axis")
            plt.grid(True)
            plt.pause(0.1)

    print("Finish")
    if show_animation:  # pragma: no cover
        plt.grid(True)
        plt.pause(1)
        plt.show()


if __name__ == '__main__':
    main()
