import numpy as np

from src.utils.logger_global import logger
from src.utils.np_vec import normalize

class CameraMover:
    def __init__(self):
        self.M_ext = M_ext = np.array([[1, 0, 0, 0],
                                       [0, 1, 0, 0],
                                       [0, 0, 1, 0],
                                       [0, 0, 0, 1]], dtype=float)

    def step_x(self, size):
        # self.M_ext[0, 3] += size
        self.M_ext[0, 3] -= size

    def step_y(self, size):
        # self.M_ext[1, 3] += size
        self.M_ext[1, 3] -= size

    def step_z(self, size):
        # self.M_ext[2, 3] += size
        self.M_ext[2, 3] -= size

    def rotate_x(self, angle):
        angle = np.radians(angle)
        R_x = np.array([[1, 0, 0, 0],
                          [0, np.cos(angle), -np.sin(angle), 0],
                          [0, np.sin(angle), np.cos(angle), 0],
                          [0, 0, 0, 1]], dtype=float)
        self.M_ext = R_x @ self.M_ext

    def rotate_y(self, angle):
        angle = np.radians(angle)
        R_y = np.array([[np.cos(angle), 0, np.sin(angle), 0],
                            [0, 1, 0, 0],
                            [-np.sin(angle), 0, np.cos(angle), 0],
                            [0, 0, 0, 1]], dtype=float)
        """
        from y axis, z axis goes right and x axis goes bottom, not up 
        """
        # R_y = np.array([[np.cos(angle), 0, -np.sin(angle), 0],
        #                     [0, 1, 0, 0],
        #                     [np.sin(angle), 0, np.cos(angle), 0],
        #                     [0, 0, 0, 1]], dtype=float)
        self.M_ext = R_y @ self.M_ext

    def rotate_z(self, angle):
        angle = np.radians(angle)
        R_z = np.array([[np.cos(angle), -np.sin(angle), 0, 0],
                          [np.sin(angle), np.cos(angle), 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]], dtype=float)
        self.M_ext = R_z @ self.M_ext

    def rotate_aim_pos_world(self, cam_pos, cam_lookat, aim_pos):
        # https://stackoverflow.com/questions/21830340/understanding-glmlookat
        z_ = aim_pos - cam_pos
        logger.debug(f"aim_pos: {aim_pos},  cam_pos: {cam_pos}, [z_]: [{z_}]")
        z_[2] *= cam_lookat[2]
        logger.debug(f"z_[2] *= cam_lookat[2] aim_pos: {aim_pos},  cam_pos: {cam_pos}, [z_]: [{z_}]")
        z_ = normalize(z_)
        logger.debug(f"z_ = normalize(z_) aim_pos: {aim_pos},  cam_pos: {cam_pos}, [z_]: [{z_}]")
        y_ = np.array([0.,1.,0.])
        # x_ = normalize(np.cross(z_, y_))
        x_ = normalize(np.cross(y_, z_))# c = np.cross(a, b) right hand. a is index finger, b is thumb. c = middle finger
        logger.debug(f"[x_, y_, z_]: [{x_},{y_},{z_}]")
        # recompute y_
        # y_ = normalize(np.cross(x_, z_))
        y_ = normalize(np.cross(z_, x_))
        logger.debug(f"[x_, y_, z_]: [{x_},{y_},{z_}]")
        # R_ = np.array([[x_[0],y_[0],z_[0], 0],
        #                [x_[1],y_[1],z_[1], 0],
        #                [x_[2],y_[2],z_[2], 0],
        #                [0, 0, 0, 1]], dtype=float)
        # R_ = np.array([[-x_[0],-y_[0],-z_[0], 0],
        #                [-x_[1],-y_[1],-z_[1], 0],
        #                [-x_[2],-y_[2],-z_[2], 0],
        #                [0, 0, 0, 1]], dtype=float)
        # self.rotate_z(90)
        logger.debug(f"cam_pos: {cam_pos}")
        # R_ = np.array([[x_[0],y_[0],z_[0], -cam_pos[0]],
        #                [x_[1],y_[1],z_[1], -cam_pos[1]],
        #                [x_[2],y_[2],z_[2], -cam_pos[2]],
        #                [0, 0, 0, 1]], dtype=float)
        R_ = np.array([[x_[0],y_[0],z_[0], 0],
                       [x_[1],y_[1],z_[1], 0],
                       [x_[2],y_[2],z_[2], 0],
                       [0, 0, 0, 1]], dtype=float)
        # self.M_ext = R_
        
        logger.debug(f"self.M_ext: {self.M_ext}")
        self.M_ext = R_ @ self.M_ext
        # self.M_ext = R_
        logger.debug(f"R_ @ self.M_ext: {self.M_ext}")

        
"""     vec1 = aim_pos - cam_pos
        vec2 = cam_lookat - cam_pos
        # vec = vec1 - vec2
        # rad = np.arctan2(vec[0], vec[1], vec[2])

        # https://qiita.com/hacchi_/items/7e6f433d465df9378d7a
        # コサインの計算
        length_vec_a = np.linalg.norm(vec1)
        length_vec_c = np.linalg.norm(vec2)
        logger.debug(f"vec1: {vec1}, vec2: {vec2}, length_a: {length_vec_a}, length_c: {length_vec_c}")
        inner_product = np.inner(vec1, vec2)
        cos = 0
        try:
            cos = inner_product / (length_vec_a * length_vec_c)
        except ZeroDivisionError as e:
            logger.debug(f"devide zero :{e}")

        # 角度（ラジアン）の計算
        rad = np.arccos(cos)
        logger.debug(f"rad: {rad}")

        angle = np.rad2deg(rad)
        self.rotate_x(angle)
        self.rotate_y(angle)
        self.rotate_z(angle)"""
        

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