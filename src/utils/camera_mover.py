import numpy as np

from src.utils.logger_global import logger

class CameraMover:
    def __init__(self):
        self.M_ext = M_ext = np.array([[1, 0, 0, 0],
                                       [0, 1, 0, 0],
                                       [0, 0, 1, 0],
                                       [0, 0, 0, 1]], dtype=float)

    def step_x(self, size):
        self.M_ext[0, 3] += size

    def step_y(self, size):
        self.M_ext[1, 3] += size

    def step_z(self, size):
        self.M_ext[2, 3] += size

    def rotate_x(self, angle):
        angle = np.radians(angle)
        R_x = np.array([[1, 0, 0, 0],
                          [0, np.cos(angle), -np.sin(angle), 0],
                          [0, np.sin(angle), np.cos(angle), 0],
                          [0, 0, 0, 1]], dtype=float)
        self.M_ext = R_x @ self.M_ext

    def rotate_y(self, angle):
        angle = np.radians(angle)
        # R_y = np.array([[np.cos(angle), 0, np.sin(angle), 0],
        #                     [0, 1, 0, 0],
        #                     [-np.sin(angle), 0, np.cos(angle), 0],
        #                     [0, 0, 0, 1]], dtype=float)
        """
        from y axis, z axis goes right and x axis goes bottom, not up 
        """
        R_y = np.array([[np.cos(angle), 0, -np.sin(angle), 0],
                            [0, 1, 0, 0],
                            [np.sin(angle), 0, np.cos(angle), 0],
                            [0, 0, 0, 1]], dtype=float)
        self.M_ext = R_y @ self.M_ext

    def rotate_z(self, angle):
        angle = np.radians(angle)
        R_z = np.array([[np.cos(angle), -np.sin(angle), 0, 0],
                          [np.sin(angle), np.cos(angle), 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]], dtype=float)
        self.M_ext = R_z @ self.M_ext

if __name__ == "__main__":
    from logger_global import logger
    mover = CameraMover()
    num_camera = 2
    for i in range(num_camera):
        camera_mover = CameraMover()
        radius = 5
        angle = np.radians(360/num_camera * i)
        step_x = radius*np.cos(angle)
        step_z = radius*np.sin(angle)
        camera_mover.step_x(step_x)
        camera_mover.step_z(step_z)
                
        # camera_mover.rotate_y(10*i)
        # camera_mover.rotate_x(20*i)
        # camera_mover.rotate_z(20*i)
        M_ext = camera_mover.M_ext

        # M_ext = np.array(frame['transform_matrix'])
        # n = 1
        # plotter.add_camera(M_ext, color='blue', name='frame'+str(i))
        # plotter.add_camera_coord(M_ext)
        # plotter.add_rays(M_ext)