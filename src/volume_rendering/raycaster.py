import numpy as np

from src.utils.logger_global import logger
from src.utils.camera_mover import CameraMover
from src.utils.camera_plotter import normalize

# poetry run python -m src.volume_rendering.raycaster

class RayCaster():
    def __init__(self, w_, h_, cam_pos, cam_lookat, cam_up, cam_right, fov = 45) -> None:
        # カメラからスクリーンまでの距離
        self.w_ = w_
        self.h_ = h_
        self.fl_x = w_ / (2. * np.tan(np.radians(fov) * 0.5))
        self.fl_y = h_ / (2. * np.tan(np.radians(fov) * 0.5))
        self.cx = (float)(w_ / 2)
        self.cy = (float)(h_ / 2)
        self.cam_lookat = cam_lookat# np.array([0, 0, 1])#([0, 0, 0])#([0, 0, -1])
        self.cam_pos = cam_pos
        self.cam_up = cam_up
        self.cam_right = cam_right

    def raycast(self, M_ext):

        # dw = (float)(2./w_)# 2 = 1.- (-1.)
        # dh = (float)(2./h_)
        # delta = 10**-5
        # lis_w = np.arange(-1., 1.+delta, 2./dw, dtype = float)
        # lis_h = np.arange(-1., 1.+delta, 2./dh, dtype = float)

        cam_pos_world = self.cam2world_cam_pos(M_ext)

        # ray_dirs = []
        ray_dirs_world = np.zeros([self.w_, self.h_, 3])
        for px in range(self.w_):
            for py in range(self.h_):
        # for px in range(-1., 1., dw):
        #     for py in range(-1., 1., dh):
        # for px in lis_w:
        #     for py in lis_h:
                # https://github.com/Rintarooo/Volume-Rendering-Tutorial/blob/f27c64f7909f368dc8205adcef2efa24b40e1e29/Tutorial1/src/VolumeRenderer.cpp#L72-L75
                # Compute the ray direction for this pixel. Same as in standard ray tracing.
				# `px`は現在考慮しているピクセルの水平方向の位置（0から`width-1`まで）、`width`は画像の幅（ピクセル数）を表しています。
				# まず`px+.5f`でピクセルの中心を基準に位置を取得し、それを`(float)(width-1)`で割ることで、0.5からwidth-1+0.5を、0から1の範囲に正規化しています。
				# その後、`-.5f`を加えることで、値の範囲を-0.5から0.5に変更しています。
                # u_ = -.5 + (px + .5) / (float)(self.w_-1)
                # v_ =  .5 - (py + .5) / (float)(self.h_-1)
                # u_ = -.5 + (px) / (float)(self.w_-1)
                # v_ =  .5 - (py) / (float)(self.h_-1)
                # u_ = -1. + (px + .5) / (float)(self.w_-1)
                # v_ =  1. - (py + .5) / (float)(self.h_-1)
                # v_ =  -1. + (py + .5) / (float)(self.h_-1)
                # u_ = -1. + (px) / (float)(self.w_-1) * 2.# 2. = 1.- (-1.), [-1. 1.]
                # v_ =  1. - (py) / (float)(self.h_-1) * 2.
                # u_ = -1. + (px+.5) / (float)(self.w_-1) * 2.# 2. = 1.- (-1.), [-1. 1.]
                # v_ =  1. - (py+.5) / (float)(self.h_-1) * 2.

                u_ = (px + .5) - self.cx
                v_ = (py + .5) - self.cy
                # u_ = px - self.cx
                # v_ = py - self.cy

                # u_ = (px + .5*dw) - cx
                # v_ = (py + .5*dh) - cy

                # logger.debug(f"[u_,v_]: [{u_},{v_}], px + .5: {px + .5}, cx:{self.cx}")
                
                ray_dir = self.cam_lookat + u_ * self.cam_right / self.fl_x + v_ * self.cam_up /self.fl_y
                # ray_dir = np.array([u_ / dist_camera2plane, v_ / dist_camera2plane, -1.])
                # ray_dir = np.array([u_ / self.fl_x, v_ / self.fl_y, 1.])
                # ray_dir = np.array([u_ / self.fl_x, v_ / self.fl_y, -1.])


                # ray_dir -= self.cam_pos

                # ray_dirs.append(ray_dir)
                # ray_dirs[self.w_-px-1][self.h_-py-1] = ray_dir

                ray_dir_world = self.cam2world_ray_dir(M_ext, ray_dir)
                ray_dir_world -= cam_pos_world
                ray_dir_world = normalize(ray_dir_world)

                ray_dirs_world[px][py] = ray_dir_world
                # logger.debug(f"[u_,v_]: [{u_},{v_}], [px,py]:[{px},{py}]")
                # logger.debug(f"ray_dir_world: {ray_dir_world}, ray_dir:{ray_dir}")
        # grid = np.mgrid[-w_/2:w_/2:1.0, -w_/2:w_/2:1.0]
        return ray_dirs_world, cam_pos_world
    
    def cam2world_ray_dir(self, M_ext, ray_dir):
        """
        input
        M_ext: 4x4 mat
        M_ext convert 3d vector x from world to cam.
        x_c = M_ext * x_w
        """
        R = M_ext[:3, :3] # rotation matrix
        t = M_ext[:3, 3] # translation vector

        ray_dir_world = R.T @ ray_dir - R.T @ t
        return ray_dir_world

    def cam2world_cam_pos(self, M_ext):
        R = M_ext[:3, :3] # rotation matrix
        t = M_ext[:3, 3] # translation vector

        cam_pos_world = R.T @ self.cam_pos - R.T @ t
        return cam_pos_world


if __name__ == "__main__":
    cam_pos = np.array([0, 0, 0])#([1.45, 1.1, 3.1])#([-1, 1, 0])
    cam_lookat = np.array([0, 0, -1])#([0, 0, 0])#([0, 0, 1])
    cam_up = np.array([0, 1, 0])
    cam_right = np.cross(cam_lookat, cam_up)
    # cam_up = np.cross(cam_right, look_dir)

    fov = 45#90#150#20#85#10#80
    w_, h_ = 3, 3#4, 4##32, 32#256,256#64,64#128, 128#1,1#5,5

    raycaster = RayCaster(w_, h_, cam_pos, cam_lookat, cam_up, cam_right, fov)
    camera_mover = CameraMover()
    M_ext = camera_mover.M_ext
    raycaster.raycast(M_ext)
    # ray_dirs = raycast(width, height, dist_cam2plane, look_dir, cam_right, cam_up, cam_pos)

