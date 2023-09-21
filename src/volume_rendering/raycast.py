
class RayCaster():
    def __init__(self, w_, h_) -> None:
        fov = 10#180
        # カメラからスクリーンまでの距離
        self.fl_x = w_ / (2. * np.tan(np.radians(fov) * 0.5))
        self.fl_y = h_ / (2. * np.tan(np.radians(fov) * 0.5))
        self.cx = (float)(w_ / 2)
        self.cy = (float)(h_ / 2)
        


    def raycast(self, w_, h_, fl_x, fl_y, look_dir, camera_right, camera_up, camera_pos):
        # cx = (float)(w_ / 2)
        # cy = (float)(h_ / 2)


        cx = (float)(w_ / 2)
        cy = (float)(h_ / 2)

        # dw = (float)(2./w_)# 2 = 1.- (-1.)
        # dh = (float)(2./h_)
        # delta = 10**-5
        # lis_w = np.arange(-1., 1.+delta, 2./dw, dtype = float)
        # lis_h = np.arange(-1., 1.+delta, 2./dh, dtype = float)


        ray_dirs = []
        for px in range(w_):
            for py in range(h_):
        # for px in range(-1., 1., dw):
        #     for py in range(-1., 1., dh):
        # for px in lis_w:
        #     for py in lis_h:
                # https://github.com/Rintarooo/Volume-Rendering-Tutorial/blob/f27c64f7909f368dc8205adcef2efa24b40e1e29/Tutorial1/src/VolumeRenderer.cpp#L72-L75
                # Compute the ray direction for this pixel. Same as in standard ray tracing.
                # u_ = -.5 + (px + .5) / (float)(w_-1)
                # v_ =  .5 - (py + .5) / (float)(h_-1)

                u_ = (px + .5) - cx
                v_ = (py + .5) - cy
                # u_ = (px + .5*dw) - cx
                # v_ = (py + .5*dh) - cy

                # logger.debug(f"u_: {u_}, v_: {v_}")
                
                # ray_dir = look_dir * dist_camera2plane + u_ * camera_right + v_ * camera_up
                # ray_dir = np.array([u_ / dist_camera2plane, v_ / dist_camera2plane, -1.])
                ray_dir = np.array([u_ / dist_camera2plane, v_ / dist_camera2plane, 1.])
                ray_dir -= camera_pos
                ray_dirs.append(ray_dir)
        # grid = np.mgrid[-w_/2:w_/2:1.0, -w_/2:w_/2:1.0]
        return ray_dirs
