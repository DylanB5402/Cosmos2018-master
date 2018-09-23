import pathfinder 

class TrajectoryFactory:

    def __init__(self, filepath, wheelbase):
        self.traj_dict = {}
        self.filepath = filepath
        self.wheelbase = wheelbase
        
    
    def add_trajectory(self, name, traj):
        self.traj_dict.update({name + "_source.traj": traj})
        tank_modifier = pathfinder.modifiers.TankModifier(traj).modify(self.wheelbase)
        self.traj_dict.update({name + "_left.traj" : tank_modifier.getLeftTrajectory()})
        self.traj_dict.update({name + "_right.traj" : tank_modifier.getRightTrajectory()})

    def save_trajectories(self):
        for filename, traj in self.traj_dict.items():
            pathfinder.serialize(self.filepath + filename, traj)

factory = TrajectoryFactory("src/main/resources/paths/", 2.09766)

factory.add_trajectory("right_to_right_switch_back", pathfinder.generate( 
                                        [pathfinder.Waypoint(0, 2.5, 0),
                                        pathfinder.Waypoint(14, 2.5, 0),
                                        pathfinder.Waypoint(22, 7.5, 0)],
                                        pathfinder.FIT_HERMITE_CUBIC, pathfinder.SAMPLES_HIGH,
                               dt=0.02, # 50WWms
                               max_velocity= 8,
                               max_acceleration=5,
                               max_jerk=100)[1])
factory.save_trajectories()

#  C:/Users/dbarv/AppData/Local/Programs/Python/Python36-32/python.exe "c:/Users/dbarv/Documents/FRC Robot Code/Cosmos2018/Cosmos2018-master/src/main/python/TrajectoryFactory.py"