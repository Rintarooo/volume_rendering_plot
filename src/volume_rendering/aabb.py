import numpy as np

from src.utils.logger_global import logger
from src.volume_rendering.cube_plotter import CubePlotter

# poetry run python -m src.volume_rendering.aabb

def swap_val(a, b):
    swap_tmp = a
    a = b
    b = swap_tmp
    return a, b

class AABB():
    def __init__(self, pos_cube_center, mint, maxt, grid_resolution) -> None:
        # self.min_x, self.min_y, self.min_z = -aabb_min, -aabb_min, -aabb_min#-1, -1, -1
        # self.max_x, self.max_y, self.max_z = aabb_max, aabb_max, aabb_max#1, 1, 1
        # self.min_x, self.min_y, self.min_z = -0.5, -0.5, -3.5#-1, -1, -1
        # self.max_x, self.max_y, self.max_z = 0.5, 0.5, -2.5#1, 1, 1

        # self.min_x, self.min_y, self.min_z = -1.0, -1.0, 6.0#-1, -1, -1
        # self.max_x, self.max_y, self.max_z = 1.0, 1.0, 8.0#1, 1, 1


        assert mint < maxt, "range cube"
        self.pos_cube_center = pos_cube_center
        self.pos_x_center, self.pos_y_center, self.pos_z_center = pos_cube_center 
        self.min_x, self.min_y, self.min_z = self.pos_x_center+mint, self.pos_y_center+mint, self.pos_z_center+mint
        self.max_x, self.max_y, self.max_z = self.pos_x_center+maxt, self.pos_y_center+maxt, self.pos_z_center+maxt
        self.min_pos = np.array([self.min_x, self.min_y, self.min_z])
        self.max_pos = np.array([self.max_x, self.max_y, self.max_z])

        # density volume
        self.grid_resolution = grid_resolution
        self.H, self.W, self.D = [self.grid_resolution for _ in range(3)]
        self.num_cell = self.H * self.W * self.D
        # initialize
        self.density_volume = np.zeros((self.H, self.W, self.D, 7))

        
    def ray_intersect_and_get_mint(self, ray_dir, camera_pos):
        # https://github.com/Rintarooo/instant-ngp/blob/090aed613499ac2dbba3c2cede24befa248ece8a/include/neural-graphics-primitives/bounding_box.cuh#L163
        # https://github.com/Southparketeer/SLAM-Large-Dense-Scene-Real-Time-3D-Reconstruction/blob/master/CUDA_Kernel/cu_raycast.cu#L127-L132
        # https://www.scratchapixel.com/lessons/3d-basic-rendering/volume-rendering-for-developers/volume-rendering-voxel-grids.html
        # https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection.html
        # print("ray_dir[0]: ", ray_dir[0])

        # max_ray_scale = 10.0
        # ray_dir *= max_ray_scale

        if_intersect = True
        # print("ray_dir: ", ray_dir)

        # r(t) = o + td, ray_dir is d
        # if ray_dir[0] != 0:# prevent divide by 0
        #     # camera_pos[0] == camera_pos.x
        #     # tmin = aabb.min_x - camera_pos[0] / ray_dir[0]
        #     tmin = (self.min_x - camera_pos[0]) / ray_dir[0]
        #     tmax = (self.max_x - camera_pos[0])/ ray_dir[0]
        # else:
        #     tmin, tmax = 100, 100#0, 0
        if ray_dir[0] == 0:
            tmin = -np.inf
            tmax = np.inf
        else:
            inv_ray_dir_x = 1. / ray_dir[0]
            tmin = (self.min_x - camera_pos[0]) * inv_ray_dir_x
            tmax = (self.max_x - camera_pos[0]) * inv_ray_dir_x

        if (tmin > tmax): tmin, tmax = swap_val(tmin, tmax)
        # print(f"tminx: {tmin}, tmaxx: {tmax}")


        # if ray_dir[1] != 0:
        #     tmin_y = (self.min_y - camera_pos[1])/ ray_dir[1]
        #     tmax_y = (self.max_y - camera_pos[1])/ ray_dir[1]
        # else:
        #     tmin_y, tmax_y = 100, 100
        if ray_dir[1] == 0:
            tmin_y = -np.inf
            tmax_y = np.inf
        else:
            inv_ray_dir_y = 1. / ray_dir[1]
            tmin_y = (self.min_y - camera_pos[1]) * inv_ray_dir_y
            tmax_y = (self.max_y - camera_pos[1]) * inv_ray_dir_y
        if (tmin_y > tmax_y): tmin_y, tmax_y = swap_val(tmin_y, tmax_y)

        if tmin > tmax_y or tmin_y > tmax:
            if_intersect = False
            return tmin, tmax, if_intersect
        # tmin = np.min([tmin, tmin_y])
        # tmax = np.max([tmax, tmax_y])
        if tmin_y > tmin: tmin = tmin_y
        if tmax_y < tmax: tmax = tmax_y
        # print(f"tmin: {tmin}, tmax: {tmax}")



        # if ray_dir[2] != 0:
        #     tmin_z = (self.min_z - camera_pos[2])/ ray_dir[2]
        #     tmax_z = (self.max_z - camera_pos[2])/ ray_dir[2]
        #     print(f"tmin_z: {tmin_z}, tmax_z: {tmax_z}")
        # else:
        #     tmin_z, tmax_z = 100, 100
        inv_ray_dir_z = 1. / ray_dir[2]
        tmin_z = (self.min_z - camera_pos[2]) * inv_ray_dir_z
        tmax_z = (self.max_z - camera_pos[2]) * inv_ray_dir_z
        if (tmin_z > tmax_z): tmin_z, tmax_z = swap_val(tmin_z, tmax_z)
        
        if tmin > tmax_z or tmin_z > tmax:
            if_intersect = False
            return tmin, tmax, if_intersect

        # tmin = np.min([tmin, tmin_z])
        # tmax = np.max([tmax, tmax_z])
        if tmin_z > tmin: tmin = tmin_z
        if tmax_z < tmax: tmax = tmax_z
        # print(f"tmin: {tmin}, tmax: {tmax}")

        return tmin, tmax, if_intersect
    
    def create_density_volume(self, thres_pos, min_density_val, max_density_val, high_density_val, density_mode):
        density = min_density_val
        for i in range(self.H):
            for k in range(self.W):
                for j in range(self.D):
                    # scaling: i is in [0, H-1] --> [0, 1] --> #[0, 2.] --> #[-1., 1.]
                    # x = i/(self.H-1)*2.+(-1.)
                    # y = k/(self.W-1)*2.+(-1.)
                    # z = i/(self.D-1)*2.+(-1.)

                    # scaling: i is in [0, H-1] --> [0, 1] --> #[0, self.max_x-self.min_x] --> #[self.min_x, slf.max_x]
                    x = i/(self.H-1)*(self.max_x-self.min_x)+self.min_x
                    y = k/(self.W-1)*(self.max_y-self.min_y)+self.min_y
                    z = j/(self.D-1)*(self.max_z-self.min_z)+self.min_z
                    density = 0.

                    if density_mode == "sphere":
                        # https://github.com/Rintarooo/Volume-Rendering-Tutorial/blob/f27c64f7909f368dc8205adcef2efa24b40e1e29/Tutorial1/src/main.cpp#L17-L21
                        radius_x = abs(x - self.pos_x_center)
                        radius_y = abs(y - self.pos_y_center)
                        radius_z = abs(z - self.pos_z_center)

                        # Fill the grid with a solid sphere with a very dense inner sphere
                        sphere_radius = radius_x * radius_x + radius_y * radius_y + radius_z * radius_z
                        if sphere_radius < 1.:
                            density = 1.0#sphere_radius
                            if sphere_radius < 0.25:
                                density = high_density_val
                    elif density_mode == "cube":
                        # range_density_val = max_density_val - min_density_val# https://stackoverflow.com/questions/59389241/how-to-generate-a-random-float-in-range-1-1-using-numpy-random-rand
                        # density = np.random.rand()*range_density_val + min_density_val
                        density = min_density_val
                        if self.pos_x_center - thres_pos <= x and x <= self.pos_x_center + thres_pos:
                            if self.pos_y_center - thres_pos <= y and y <= self.pos_y_center + thres_pos:
                                if self.pos_z_center - thres_pos <= z and z <= self.pos_z_center + thres_pos:
                                    # logger.debug(f"high_density_val. because x, y and z is in range min z: {self.pos_z_center - thres_pos} <= z: {z} <= max z: {self.pos_z_center + thres_pos}")
                                    density = high_density_val

                    # white
                    r, g, b = 255, 255, 255

                    # self.density_volume[i][k][j] = (x,y,z,density)
                    
                    # 3d position
                    self.density_volume[i][k][j][0] = x
                    self.density_volume[i][k][j][1] = y
                    self.density_volume[i][k][j][2] = z
                    # rgb
                    self.density_volume[i][k][j][3] = r
                    self.density_volume[i][k][j][4] = g
                    self.density_volume[i][k][j][5] = b
                    # density
                    self.density_volume[i][k][j][6] = density
                    # logger.debug(f"i: {i}, k: {k}, j: {j}")
                    # logger.debug(f"cell pos, x: {x}, y: {y}, z: {z}, density: {density}")

    def read_volume(self, point_pos):
        # nearest neighbor search
        # https://www.scratchapixel.com/lessons/3d-basic-rendering/volume-rendering-for-developers/volume-rendering-voxel-grids.html
        # https://github.com/Rintarooo/code/blob/7f5ea4b995026e9fd1fba127ebce059830e95584/volume-rendering-for-developers/raymarch-chap5.cpp#L247-L259
    #     float evalDensity(const Grid* grid, const Point& p)
    # {
        gridSize_xyz = self.max_pos - self.min_pos
        pLocal = (point_pos - self.min_pos) / gridSize_xyz
        pVoxel = pLocal * self.grid_resolution

        # relative index
        xi =int(np.floor(pVoxel[0]))
        yi =int(np.floor(pVoxel[1]))
        zi =int(np.floor(pVoxel[2]))

        # If outside the grid, then return density zero
        if (xi < 0) or (xi >= self.W): return 0
        if (yi < 0) or (yi >= self.H): return 0
        if (zi < 0) or (zi >= self.D): return 0


        ## nearest neighbor
        # grid_idx = (zi * grid->resolution + yi) * grid->resolution + xi
        # return grid->density[grid_idx]
        x = self.density_volume[xi][yi][zi][0]
        y = self.density_volume[xi][yi][zi][1]
        z = self.density_volume[xi][yi][zi][2]
        density = self.density_volume[xi][yi][zi][6]
        # if point_pos[0] == 0 and point_pos[1]==0:
            # logger.debug(f"\nxi: {xi}, yi: {yi}, zi: {zi}")
        logger.debug(f"\nray point_pos: {point_pos}\ncell pos: [{x}, {y}, {z}]\nread density: {density}")
        return density
    # }

if __name__ == "__main__":
    pos_cube_center = (0,0,9)#(0,0,7)#(0,0,-3)
    mint, maxt = -2.0, 2.0#-3.0, 3.0#-1.0, 1.0#-0.5, 0.5
    rgba = 'rgba(0,0,0,0.2)'
    cube_plotter = CubePlotter(pos_cube_center, mint, maxt, rgba)
    trace_cube = cube_plotter.get_trace_cube()

    grid_resolution = 16#128#30
    aabb = AABB(pos_cube_center, mint, maxt, grid_resolution)
