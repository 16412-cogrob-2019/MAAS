import numpy as np 
from bayes_opt import BayesianOptimization, UtilityFunction
# from caldera_mcts import *
# from utils import *
# import matplotlib.mlab as mlab
from copy import deepcopy

'''
dim_x, dim_y = 100, 100
pbounds = {'x': (0, dim_x), 'y': (0, dim_y)}

optimizer = BayesianOptimization(f=caldera_sim_function, pbounds=pbounds, verbose=2, random_state=1)
utility = UtilityFunction(kind="ucb", kappa=100, xi=0.0)
num_iterations = 10
num_suggested_points = 7
num_robots = 3
xs = [[] for _ in range(num_robots)]
ys = [[] for _ in range(num_robots)]


for i in range(num_iterations):

	print('Iteration', i)
	optimizer_ = deepcopy(optimizer)
	points_to_suggest = []

	for j in range(num_suggested_points):

		print('Sample', j)
		next_point = optimizer_.suggest(utility)
		next_point_arr = [next_point['x'], next_point['y']]
		next_point_arr_np = np.atleast_2d(np.array(next_point_arr))

		target = optimizer_._gp.predict(next_point_arr_np)
		target = target[0]

		optimizer_.register(params=next_point, target=target)
		points_to_suggest.append(next_point)

	indices = np.random.choice(num_suggested_points, num_robots, replace=False)
	final_points = [points_to_suggest[k] for k in indices]

	print(points_to_suggest)
	print(final_points)

	for j, p in enumerate(final_points):

		target = caldera_sim_function(**p)
		optimizer.register(params=p, target=target)
		xs[j].append(p['x'])
		ys[j].append(p['y'])

for i in range(num_robots):
	plot_trajectory(xs[i][1:], ys[i][1:], dim_x, dim_y)

'''

def suggest_points(num_suggested_points, utility_fn, pbounds, samples):
	optimizer = BayesianOptimization(f=lambda x, y: 0, pbounds=pbounds, verbose=2, random_state=1)
	for s in samples:
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

'''
for i in range(num_iterations):
	print(i)
	next_point = optimizer.suggest(utility)
	target = caldera_sim_function(**next_point)
	print(target)
	optimizer.register(params=next_point, target=target)
	xs.append(next_point['x'])
	ys.append(next_point['y'])

plot_trajectory(xs[1:], ys[1:], dim_x, dim_y)
print("Maximum found depth is %.2f, found at coordinates x: %.1f y:%.1f" % (optimizer.max['target'], optimizer.max['params']['x'], optimizer.max['params']['y']))
'''