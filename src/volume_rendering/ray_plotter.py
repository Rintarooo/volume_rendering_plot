import numpy as np
import plotly.graph_objects as go

from src.utils.logger_global import logger
from src.utils.base_plotter import BasePlotter
from src.utils.camera_plotter import CameraPlotter
from src.utils.camera_mover import CameraMover
from src.volume_rendering.raycaster import RayCaster


# poetry run python -m src.volume_rendering.ray_plotter

class RayPlotter(BasePlotter):
    # def __init__(self, w_, h_, fov = 45):
    def __init__(self):
        super().__init__()

        # カメラからスクリーンまでの距離
        # self.fl_x = w_ / (2. * np.tan(np.radians(fov) * 0.5))
        # self.fl_y = h_ / (2. * np.tan(np.radians(fov) * 0.5))
        # self.cx = (float)(w_ / 2)
        # self.cy = (float)(h_ / 2)
        
        # # カメラからスクリーンまでの距離
        # if use_transforms_json:
        #     self.fl_x = transforms["fl_x"]
        #     self.fl_y = transforms["fl_y"]
        #     self.cx = transforms["cx"]
        #     self.cy = transforms["cy"]
            
        self.ray_dirs_list = []
    

    def get_traces_ray(self, cam_pos, ray_dir, tmax, tmax_margin, plot_name):
        # tmax = 1.0
        # logger.debug(f"cam_pos: {cam_pos}, ray_dir_0: {ray_dir_0}")
        ray_ = cam_pos + (tmax+tmax_margin) * ray_dir
        trace1 = {
            "line": {"width": 3}, 
            "mode": "lines", 
            "name": plot_name,#"ray_", 
            "type": "scatter3d", 
            "x": [cam_pos[0], ray_[0]], 
            "y": [cam_pos[1], ray_[1]], 
            "z": [cam_pos[2], ray_[2]], 
            "marker": {"color": "rgb(255, 217, 0)"}, 
            # "marker": {"line": {"color": "rgb(35, 155, 118)"}}, 
            "showlegend": False
        }
        trace2 = {
            "name": plot_name, #"ray_", 
            "type": "cone", 
            "u": [ray_[0]-cam_pos[0]], # 矢印の終点のx座標
            "v": [ray_[1]-cam_pos[1]], 
            "w": [ray_[2]-cam_pos[2]], 
            "x": [ray_[0]], # 矢印の始点のx座標
            "y": [ray_[1]], 
            "z": [ray_[2]], 
            "sizeref": 0.05, 
            "lighting": {"ambient": 0.8}, 
            "sizemode": "scaled", 
            "hoverinfo": "x+y+z+name", 
            "colorscale": [[0.0, 'rgb(255,217,0)'],[1.0, 'rgb(255,217,0)']],
            "showscale": False, 
            "autocolorscale": False
        }
        traces_ray = [trace1, trace2]
        return traces_ray

    def get_trace_sampling_point(self, sampling_points):
        trace1 = {
            "name": "sampling_point", 
            "type": "scatter3d", 
            "x": sampling_points[:,0], # 点のx座標
            "y": sampling_points[:,1], 
            "z": sampling_points[:,2], 
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
        # trace_sampling_point = [trace1]
        # return trace_sampling_point
        return trace1


if __name__ == "__main__":
    cam_pos = np.array([0, 0, 0])#([1.45, 1.1, 3.1])#([-1, 1, 0])
    cam_lookat = np.array([0, 0, -1])#([0, 0, 0])#([0, 0, 1])
    cam_up = np.array([0, 1, 0])
    cam_right = np.cross(cam_lookat, cam_up)
    # cam_up = np.cross(cam_right, look_dir)

    fov = 45#90#150#20#85#10#80
    img_resolution = 12
    w_, h_ = img_resolution, img_resolution#3, 3#4, 4##32, 32#256,256#64,64#128, 128#1,1#5,5
    plot_scale = 1.0
    cam_plotter = CameraPlotter(w_, h_, cam_pos, cam_lookat, cam_up, fov, plot_scale)
    camera_mover = CameraMover()
    # camera_mover.step_x(-2)
    # camera_mover.rotate_x(10)
    M_ext = camera_mover.M_ext
                
    plot_name = 'debug raycast'
    cam_plotter.add_cam(M_ext, plot_color='blue', plot_name=plot_name)
    cam_plotter.add_trace_cam_coord()
    cam_plotter.add_trace_cam_screen()

    raycaster = RayCaster(w_, h_, cam_lookat, cam_pos, cam_up, cam_right, fov)
    ray_dirs_world, cam_pos_world = raycaster.raycast(M_ext)
    # i = 0
    # ray_dir_world = ray_dirs_world[i][i]
    for px in range(w_):
        for py in range(h_):
            ray_dir_world = ray_dirs_world[px][py]
            ray_plotter = RayPlotter()
            traces_ray = ray_plotter.get_traces_ray(cam_pos_world, ray_dir_world, plot_name)
            # ray_plotter.fig.add_traces(traces_ray)
            # ray_plotter.fig_show()
            cam_plotter.fig.add_traces(traces_ray)
    cam_plotter.fig_show()




    """bash
    cam_pos = np.array([0, 0, 0])#([1.45, 1.1, 3.1])#([-1, 1, 0])
    cam_lookat = np.array([0, 0, -1])#([0, 0, 0])#([0, 0, 1])
    cam_up = np.array([0, 1, 0])
    cam_right = np.cross(cam_lookat, cam_up)
    # cam_up = np.cross(cam_right, look_dir)

    fov = 45#90#150#20#85#10#80
    width, height = 32, 32#256,256#64,64#3, 3#128, 128#1,1#5,5

    ray_dirs = raycast(width, height, dist_cam2plane, look_dir, cam_right, cam_up, cam_pos)
    
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
        tmin, tmax, if_intersect = aabb.ray_intersect_and_get_mint(ray_dir, cam_pos)
        if not if_intersect:
            tmin = tmin_0
            tmax = tmax_0
        assert tmin < tmax, "ray marching range"
        ray_max = cam_pos + (tmax+3.0) * ray_dir
        # for plotly
        trace_ray = get_trace_ray(cam_pos, ray_max, "ray_dir_"+str(ray_idx))
        trace_ray_lis.append(trace_ray)
        if if_intersect:
            dt = (tmax-tmin)/num_point
            # r(t) = o + td = o + (tmin+dt*i)d
            sampling_point = np.array([cam_pos + (tmin + dt*i) * ray_dirs[ray_idx] for i in range(num_point)])
            trace_sampling_point = get_trace_sampling_point(sampling_point)
            trace_sampling_point_lis.append(trace_sampling_point)
    """