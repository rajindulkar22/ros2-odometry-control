
class DiffDriveRobot:
    def __init__(self, wheel_radius, wheel_base):
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base

#Forward Kinematics (Wheels to Robot):
    
    def forward_kinematics(self, v_left, v_right): 
        linear_vel = self.wheel_radius * (v_right + v_left) / 2
        angular_vel = self.wheel_radius * (v_right - v_left) / self.wheel_base
        return linear_vel, angular_vel

#Inverse Kinematics (Robot to  Wheels):
    
    def inverse_kinematics(self, linear_vel, angular_vel):
        v_right = (((linear_vel + (angular_vel * self.wheel_base/2)) / self.wheel_radius))
        v_left = (((linear_vel - (angular_vel * self.wheel_base/2)) / self.wheel_radius))
        return v_right, v_left

robot = DiffDriveRobot(wheel_radius = 0.05, wheel_base = 0.2)
#test 1 : moving straight at 1.0 m/s
wl,wr = robot.inverse_kinematics( linear_vel = 1.0, angular_vel = 0.0)
print(f"straight left wheel speed: {wl} right wheel speed: {wr}")
#test 2 : moving in circle at 1.0 m/s
wl,wr = robot.inverse_kinematics( linear_vel = 0.0, angular_vel = 1.0)
print(f"circle left wheel speed: {wl} right wheel speed: {wr}")







        
        

        