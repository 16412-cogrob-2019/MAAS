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
                                                 random_state=optimizer._gp._random_state)

    for s in samples:
        if s not in list(optimizer._space._params):
            optimizer.register(params=s[0], target=s[1])

    points_to_suggest = []

    for i in range(num_suggested_points):
        print('Sample', i)
        next_point = optimizer.suggest(utility_fn)
        next_point_arr = [next_point['x'], next_point['y']]
        next_point_arr_np = np.atleast_2d(np.array(next_point_arr))

        mu = optimizer._gp.predict(next_point_arr_np)
        target = mu[0]

        optimizer.register(params=next_point, target=target)
        point_with_reward = next_point.copy()
        point_with_reward['reward'] = utility_fn.utility(next_point_arr_np, optimizer._gp, 0)[0]
        points_to_suggest.append(point_with_reward)

    return points_to_suggest


def suggest_points_with_std(num_suggested_points, utility_fn, pbounds, samples, use_new_kernel=True):
    optimizer = BayesianOptimization(f=lambda x, y: 0, pbounds=pbounds, verbose=2, random_state=1)
    if use_new_kernel:
        optimizer._gp = GaussianProcessRegressor(kernel=50. * RBF(length_scale=0.1),
                                                 alpha=1e-6,
                                                 normalize_y=True,
                                                 n_restarts_optimizer=25,
                                                 random_state=optimizer._gp._random_state)

    for s in samples:
        if s not in list(optimizer._space._params):
            optimizer.register(params=s[0], target=s[1])

    points_to_suggest = []

    for i in range(num_suggested_points):
        print('Sample', i)
        next_point = optimizer.suggest(utility_fn)
        next_point_arr = [next_point['x'], next_point['y']]
        next_point_arr_np = np.atleast_2d(np.array(next_point_arr))

        mu, std = optimizer._gp.predict(next_point_arr_np, return_std=True)
        mu, std = mu[0], std[0]
        target = np.random.normal(mu, std)

        optimizer.register(params=next_point, target=target)
        point_with_reward = next_point.copy()
        point_with_reward['reward'] = utility_fn.utility(next_point_arr_np, optimizer._gp, 0)[0]
        points_to_suggest.append(point_with_reward)

    return points_to_suggest


def suggest_points_ts(num_suggested_points, utility_fn, pbounds, samples, num_test_points=1000, use_new_kernel=True):
    optimizer = BayesianOptimization(f=lambda x, y: 0, pbounds=pbounds, verbose=2, random_state=1)
    if use_new_kernel:
        optimizer._gp = GaussianProcessRegressor(kernel=50. * RBF(length_scale=0.1),
                                                 alpha=1e-6,
                                                 normalize_y=True,
                                                 n_restarts_optimizer=25,
                                                 random_state=optimizer._gp._random_state)

    for s in samples:
        if s not in list(optimizer._space._params):
            optimizer.register(params=s[0], target=s[1])

    points_to_suggest = []

    for i in range(num_suggested_points):
        print('Sample', i)

        test_points = np.array([[np.random.uniform(pbounds['x'][0], pbounds['x'][1]),
                                 np.random.uniform(pbounds['y'][0], pbounds['y'][1])] for _ in range(num_test_points)])

        test_points_vals = gp_draw_samples(1, test_points, optimizer)[0]
        idx = np.argmax(test_points_vals)
        selected_point = test_points[idx]
        reward = utility_fn.utility(np.atleast_2d(selected_point), optimizer._gp, 0)[0]
        points_to_suggest.append({'x': selected_point[0], 'y': selected_point[1], 'reward': reward})

    return points_to_suggest


def gp_draw_samples(num_samples, x_test, optimizer):
    mean, cov = optimizer._gp.predict(x_test, return_cov=True)
    return draw_gaussian_samples(num_samples, mean, cov)


def draw_gaussian_samples(num_samples, mu, K):
    num_pts = len(mu)
    L = stable_cholesky(K)
    U = np.random.normal(size=(num_pts, num_samples))
    V = L.dot(U).T + mu
    return V


def stable_cholesky(M):
    """ Returns L, a 'stable' cholesky decomposition of M. L is lower triangular and
        satisfies L*L' = M.
        Sometimes nominally psd matrices are not psd due to numerical issues. By adding a
        small value to the diagonal we can make it psd. This is what this function does.
        Use this iff you know that K should be psd. We do not check for errors
    """
    # pylint: disable=superfluous-parens
    if M.size == 0:
        return M  # if you pass an empty array then just return it.
    try:
        # First try taking the Cholesky decomposition.
        L = np.linalg.cholesky(M)
    except np.linalg.linalg.LinAlgError:
        # If it doesn't work, then try adding diagonal noise.
        diag_noise_power = -11
        max_M = np.diag(M).max()
        diag_noise = np.diag(M).max() * 1e-11
        chol_decomp_succ = False
        while not chol_decomp_succ:
            try:
                L = np.linalg.cholesky(M + (10 ** diag_noise_power * max_M) * np.eye(M.shape[0]))
                chol_decomp_succ = True
            except np.linalg.linalg.LinAlgError:
                diag_noise_power += 1
        if diag_noise_power >= 5:
            print('**************** Cholesky failed: Added diag noise = %e' % (diag_noise))
    return L


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
        suggested_points = suggest_points_with_std(num_suggested_points, utility, pbounds, samples,
                                                   use_new_kernel=False)
        indices = np.random.choice(num_suggested_points, num_robots, replace=False)
        final_points = [suggested_points[k] for k in indices]

        print(suggested_points)
        print(final_points)

        for j, p in enumerate(final_points):
            del p['reward']
            target = caldera_sim_function(**p)
            samples.append((p, target))
            xs[j].append(p['x'])
            ys[j].append(p['y'])

    for i in range(num_robots):
        plot_trajectory(xs[i][1:], ys[i][1:], dim_x, dim_y)

    xss = [xs_[1:] for xs_ in xs]
    yss = [ys_[1:] for ys_ in ys]

    plot_trajectories(xss, yss, dim_x, dim_y)
