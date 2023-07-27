import pickle
import datetime
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

from collections import defaultdict

sns.set(font_scale=1.5, rc={'text.usetex' : True})
sns.set_style(style='white')

def loadPickles(fname):
	ret = None
	with open(fname, 'rb') as f:
		while True:
			try:
				ret = pickle.load(f)
				break

			except EOFError:
				break

	return ret

def def_val():
    return []

def extract_data(data):

    solver_status_data = data['/solverState']
    odom_data = data['/gmapping/odometry']
    
    times = []
    positions = []
    velocities = []

    fail_points = defaultdict(def_val)

    start_t = datetime.datetime.fromtimestamp(odom_data[0][0].to_sec())
    prev_t = start_t 
    prev_x = odom_data[0][1].pose.pose.position.x
    prev_y = odom_data[0][1].pose.pose.position.y

    last_status_ind = 0

    for ind, (timestamp, msg) in enumerate(odom_data):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
	
        t = datetime.datetime.fromtimestamp(timestamp.to_sec())

        # for i in range(last_status_ind, len(solver_status_data)):
        #     stamp, m = solver_status_data[i]
        #     if (datetime.datetime.fromtimestamp(stamp)-t).total_seconds() < 0:
        #         continue
        #     else:
        #         last_status_ind = i
        #         break

        dt = (t-prev_t).total_seconds()
        if dt < 1e-6:
            prev_t = t
            continue
        
        times.append((t-start_t).total_seconds())
        positions.append([x,y])
        
        vel_x = (x-prev_x)*30
        vel_y = (y-prev_y)*30
        velocities.append([vel_x, vel_y])
        
        prev_t = t
	
    for ind, (timestamp, msg) in enumerate(solver_status_data):
	    
        if msg.status.data != 0:
            fail_points[msg.status.data-1].append(msg.initialPVA.positions[:2])

    obs_data = data['/paddedObs'][-1]
    obs_points = []
    for p in obs_data[1].points:
        obs_points.append([p.x, p.y])

    return np.array(obs_points), np.array(positions), np.array(velocities), fail_points

def main():
    data = loadPickles("/home/nick/catkin_ws/src/robust_fast_navigation/bags/occluded_trap_1.0_full_success.pkl")

    obs_points, poses, vels, fail_points = extract_data(data)

    # print(fail_points)

    plt.scatter(obs_points[:,0], obs_points[:,1], color='grey', s=1)
    plt.plot(poses[:,0], poses[:,1],linewidth=3)
    
    cols = np.array(['r', 'g', 'black', 'cyan'])
    labels = np.array(['JPS', 'Corridor', 'No Solution', 'Const. Viol.'])
    for ind, (color, label) in enumerate(zip(cols, labels)):
        points = np.array(fail_points[ind])
        if points.shape[0] > 0:
            plt.scatter(points[:,0], points[:,1], marker='^', c=color, label=label,s=70)

    plt.legend()
    plt.show()

if __name__ == "__main__":
	main()
