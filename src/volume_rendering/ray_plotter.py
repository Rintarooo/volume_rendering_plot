import numpy as np
import plotly.graph_objects as go

from src.utils.logger_global import logger
from src.utils.base_plotter import BasePlotter

# poetry run python -m src.volume_rendering.ray_plotter

class RayPlotter(BasePlotter):
    def __init__(self, w_, h_, fov):
        super().__init__()

        fov = 10#180
        # カメラからスクリーンまでの距離
        self.fl_x = w_ / (2. * np.tan(np.radians(fov) * 0.5))
        self.fl_y = h_ / (2. * np.tan(np.radians(fov) * 0.5))
        self.cx = (float)(w_ / 2)
        self.cy = (float)(h_ / 2)
        
        # # カメラからスクリーンまでの距離
        # if use_transforms_json:
        #     self.fl_x = transforms["fl_x"]
        #     self.fl_y = transforms["fl_y"]
        #     self.cx = transforms["cx"]
        #     self.cy = transforms["cy"]
            
        self.ray_dirs_list = []
    

    def get_trace_ray(self, camera_pos, ray_dir_0, name):
        trace1 = {
            "line": {"width": 3}, 
            "mode": "lines", 
            "name": name,#"ray_dir_0", 
            "type": "scatter3d", 
            "x": [camera_pos[0], ray_dir_0[0]], 
            "y": [camera_pos[1], ray_dir_0[1]], 
            "z": [camera_pos[2], ray_dir_0[2]], 
            "marker": {"color": "rgb(255, 217, 0)"}, 
            # "marker": {"line": {"color": "rgb(35, 155, 118)"}}, 
            "showlegend": False
        }
        trace2 = {
            "name": name, #"ray_dir_0", 
            "type": "cone", 
            "u": [ray_dir_0[0]-camera_pos[0]], # 矢印の終点のx座標
            "v": [ray_dir_0[1]-camera_pos[1]], 
            "w": [ray_dir_0[2]-camera_pos[2]], 
            "x": [ray_dir_0[0]], # 矢印の始点のx座標
            "y": [ray_dir_0[1]], 
            "z": [ray_dir_0[2]], 
            "sizeref": 0.1, 
            "lighting": {"ambient": 0.8}, 
            "sizemode": "scaled", 
            "hoverinfo": "x+y+z+name", 
            "colorscale": [[0.0, 'rgb(255,217,0)'],[1.0, 'rgb(255,217,0)']],
            "showscale": False, 
            "autocolorscale": False
        }
        trace_ray = [trace1, trace2]
        return trace_ray

    def get_trace_sampling_point(self, ray_dir_0_sample):
        trace1 = {
            "name": "sampling_point", 
            "type": "scatter3d", 
            "x": ray_dir_0_sample[:,0], # 点のx座標
            "y": ray_dir_0_sample[:,1], 
            "z": ray_dir_0_sample[:,2], 
            # "sizeref": 0.1, 
            # "lighting": {"ambient": 0.8}, 
            # "sizemode": "scaled", 
            # "hoverinfo": "x+y+z+name", 
            # "colorscale": [[0.0, 'rgb(0,0,0)'],[1.0, 'rgb(0,0,0)']],
            # "showscale": False, 
            # "autocolorscale": False
            "mode": 'markers',
            "marker": {
                "size": 4,
                "color": 'rgb(0,0,0)',
                "colorscale": 'Viridis',
                "opacity": 0.2
            }
        }
        trace_sampling_point = [trace1]
        return trace_sampling_point


if __name__ == "__main__":
    camera_pos = np.array([0, 0, 0])#([1.45, 1.1, 3.1])#([-1, 1, 0])
    camera_lookat = np.array([0, 0, 1])#([0, 0, 0])#([0, 0, -1])
    camera_up = np.array([0, 1, 0])
    camera_right = np.cross(look_dir, camera_up)
    # camera_up = np.cross(camera_right, look_dir)

    fov = 45#90#150#20#85#10#80
    width, height = 32, 32#256,256#64,64#3, 3#128, 128#1,1#5,5

    ray_dirs = raycast(width, height, dist_camera2plane, look_dir, camera_right, camera_up, camera_pos)
    
    pos_cube_center = (0,0,9)#(0,0,7)#(0,0,-3)
    mint, maxt = -2.0, 2.0#-3.0, 3.0#-1.0, 1.0#-0.5, 0.5
    rgba = 'rgba(0,0,0,0.2)'
    trace_cube = get_trace_cube(pos_cube_center, mint, maxt, rgba)
    grid_resolution = 16#128#30
    aabb = AABB(pos_cube_center, mint, maxt, grid_resolution)

    ray_idx_plot_lis = [0, int(len(ray_dirs)/2), int(len(ray_dirs))-1]#[0, int(len(ray_dirs)/4), int(len(ray_dirs)/2), int(len(ray_dirs)*3/4), int(len(ray_dirs))-1]#[0,4,8]

    # ray cast
    tmin_0 = 4.5#3.5
    tmax_0 = 9.5#4.5
    tmin = tmin_0
    tmax = tmax_0
    assert tmin < tmax, "ray marching range"
    trace_ray_lis = []
    trace_sampling_point_lis = []

    num_point = 5#128#30#10
    for ray_idx in ray_idx_plot_lis:
        assert ray_idx <= len(ray_dirs), "ray index should be lower than ray number (render img resolution: H x W)"
        ray_dir = ray_dirs[ray_idx]
        tmin, tmax, if_intersect = aabb.ray_intersect_and_get_mint(ray_dir, camera_pos)
        if not if_intersect:
            tmin = tmin_0
            tmax = tmax_0
        assert tmin < tmax, "ray marching range"
        ray_max = camera_pos + (tmax+3.0) * ray_dir
        # for plotly
        trace_ray = get_trace_ray(camera_pos, ray_max, "ray_dir_"+str(ray_idx))
        trace_ray_lis.append(trace_ray)
        if if_intersect:
            dt = (tmax-tmin)/num_point
            # r(t) = o + td = o + (tmin+dt*i)d
            sampling_point = np.array([camera_pos + (tmin + dt*i) * ray_dirs[ray_idx] for i in range(num_point)])
            trace_sampling_point = get_trace_sampling_point(sampling_point)
            trace_sampling_point_lis.append(trace_sampling_point)
