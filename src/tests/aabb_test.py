import numpy as np
import plotly.graph_objects as go

from src.utils.logger_global import logger
from src.utils.camera_plotter import CameraPlotter
from src.utils.camera_mover import CameraMover

from src.volume_rendering.cube_plotter import CubePlotter
from src.volume_rendering.ray_plotter import RayPlotter
from src.volume_rendering.raycaster import RayCaster
from src.volume_rendering.aabb import AABB

# poetry run python -m src.tests.aabb_test

if __name__ == "__main__":
    plot_scale = 1.0
    cam_pos = np.array([0, 0, 0])
    cam_lookat = np.array([0, 0, -1])
    cam_up = np.array([0, 1, 0])
    cam_right = np.cross(cam_lookat, cam_up)
    fov = 30#45
    w_, h_ = 3, 3#4, 4##32, 32#256,256#64,64#128, 128#1,1#5,5

    cam_plotter = CameraPlotter(w_, h_, cam_pos, cam_lookat, cam_up, fov, plot_scale)
    raycaster = RayCaster(w_, h_, cam_pos, cam_lookat, cam_up, cam_right, fov)
    ray_plotter = RayPlotter()

    pos_cube_center = (0,0,0)#(0,0,9)#(0,0,7)#(0,0,-3)
    mint, maxt = -1.0, 1.0#-2.0, 2.0#-3.0, 3.0#-0.5, 0.5
    rgba = 'rgba(0,0,0,0.2)'
    cube_plotter = CubePlotter(pos_cube_center, mint, maxt, rgba)
    trace_cube = cube_plotter.get_trace_cube()
    cam_plotter.fig.add_trace(trace_cube)

    grid_resolution = 16#128#30
    aabb = AABB(pos_cube_center, mint, maxt, grid_resolution)

    
    num_cam = 3
    for i in range(num_cam):

        cam_mover = CameraMover()
        
        radius = 4
        degree = 360./num_cam * i
        angle = np.radians(degree)
        step_x = radius*np.cos(angle)
        step_z = radius*np.sin(angle)
        cam_mover.step_x(step_x)
        cam_mover.step_z(step_z)
        cam_mover.rotate_y(-360./num_cam * i - 90)

        M_ext = cam_mover.M_ext
        cam_plotter.add_cam(M_ext, plot_color='blue', plot_name='world')
        cam_plotter.add_trace_cam_coord()
        # cam_plotter.add_trace_cam_screen()

        ray_dirs_world, cam_pos_world = raycaster.raycast(M_ext)
        # ray_idx_plot_lis = [0, int(len(ray_dirs)/2), int(len(ray_dirs))-1]#[0, int(len(ray_dirs)/4), int(len(ray_dirs)/2), int(len(ray_dirs)*3/4), int(len(ray_dirs))-1]#[0,4,8]

        # ray cast
        tmin_0 = 4.5#3.5
        tmax_0 = 9.5#4.5
        tmin = tmin_0
        tmax = tmax_0
        assert tmin < tmax, "ray marching range"
        trace_ray_lis = []
        trace_sampling_point_lis = []

        num_point = 10#5#128#30
        tmax_margin = 2.0
        for px in range(w_):
            for py in range(h_):
            # for ray_idx in ray_idx_plot_lis:
                ray_dir_world = ray_dirs_world[px][py]

                # assert ray_idx <= len(ray_dirs), "ray index should be lower than ray number (render img resolution: H x W)"
                # ray_dir = ray_dirs[ray_idx]
                tmin, tmax, if_intersect = aabb.ray_intersect_and_get_mint(ray_dir_world, cam_pos_world)
                if not if_intersect:
                    tmin = tmin_0
                    tmax = tmax_0
                assert tmin < tmax, "ray marching range"
                # for plotly
                ray_idx = py*h_+px
                traces_ray = ray_plotter.get_traces_ray(cam_pos_world, ray_dir_world, tmax, tmax_margin, plot_name= "ray"+str(ray_idx))
                # ray_plotter.fig.add_traces(traces_ray)
                # ray_plotter.fig_show()
                cam_plotter.fig.add_traces(traces_ray)

                if if_intersect:
                    dt = (tmax-tmin)/num_point
                    # r(t) = o + td = o + (tmin+dt*i)d
                    sampling_points = np.array([cam_pos_world + (tmin + dt*i) * ray_dir_world for i in range(num_point)])
                    trace_sampling_points = ray_plotter.get_trace_sampling_point(sampling_points)
                    cam_plotter.fig.add_trace(trace_sampling_points)
        
    cam_plotter.fig_show()
