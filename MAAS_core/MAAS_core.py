import numpy as np
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
from bayes_opt import BayesianOptimization, UtilityFunction


def suggest_points(num_suggested_points, utility_fn, pbounds, samples, use_new_kernel=True):
    optimizer = BayesianOptimization(f=lambda x, y: 0, pbounds=pbounds, verbose=2, random_state=1)
    if use_new_kernel:
        optimizer._gp = GaussianProcessRegressor(kernel=50. * RBF(length_scale=0.1),
                                                 alpha=1e-6,
                                                 normalize_y=True,
                                                 n_restarts_optimizer=25,
                                                 random_state=optimizer._random_state)
    for s in samples:
        if s not in list(optimizer._space._params):
            optimizer.register(params=s[0], target=s[1])

    points_to_suggest = []

    for i in range(num_suggested_points):
        # print('Sample', i)
        next_point = optimizer.suggest(utility_fn)
        next_point_arr = [next_point['x'], next_point['y']]
        next_point_arr_np = np.atleast_2d(np.array(next_point_arr))

        target, std = optimizer._gp.predict(next_point_arr_np, return_std=True)
        target = target[0]

        optimizer.register(params=next_point, target=target)
        point_with_reward = next_point.copy()
        point_with_reward['reward'] = utility_fn.utility(next_point_arr_np, optimizer._gp, 0)[0]
        points_to_suggest.append(point_with_reward)

    return points_to_suggest


if __name__ == '__main__':

    dim_x, dim_y = 100, 100
    pbounds = {'x': (0, dim_x), 'y': (0, dim_y)}
    num_iterations = 10
    num_suggested_points = 6
    num_robots = 3
    xs = [[] for _ in range(num_robots)]
    ys = [[] for _ in range(num_robots)]
    samples = []
    utility = UtilityFunction(kind="ucb", kappa=100, xi=0.0)

    for i in range(num_iterations):

        print('Iteration', i)
        suggested_points = suggest_points(num_suggested_points, utility, pbounds, samples)
        indices = np.random.choice(num_suggested_points, num_robots, replace=False)
        final_points = [suggested_points[k] for k in indices]

        print(suggested_points)
        print(final_points)

        for j, p in enumerate(final_points):
            target = caldera_sim_function(**p)
            samples.append((p, target))
            xs[j].append(p['x'])
            ys[j].append(p['y'])

    for i in range(num_robots):
        plot_trajectory(xs[i][1:], ys[i][1:], dim_x, dim_y)