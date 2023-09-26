import numpy as np
import plotly.graph_objects as go
import sys

from src.utils.logger_global import logger
from src.utils.get_yaml import YamlLoader
from src.utils.camera_plotter import CameraPlotter
from src.utils.camera_mover import CameraMover

from src.volume_rendering.cube_plotter import CubePlotter
from src.volume_rendering.ray_plotter import RayPlotter
from src.volume_rendering.ray_caster import RayCaster
from src.volume_rendering.aabb import AABB
from src.volume_rendering.volume_renderer import render_img_from_volume


if __name__ == "__main__":
    logger.debug(f"len(sys.argv): {len(sys.argv)}")
    assert len(sys.argv) == 2, "specify yaml file\npoetry run python -m main config/cfg.yaml"
    yaml_loader = YamlLoader(sys.argv[1])
    cfg = yaml_loader.load_yaml()

    cam_pos = np.array(cfg["cam_pos"])
    cam_lookat = np.array(cfg["cam_lookat"])
    cam_up = np.array(cfg["cam_up"])
    cam_right = np.cross(cam_lookat, cam_up)

    fov = cfg["fov"]
    w_, h_ = cfg["w_"], cfg["h_"]

    cam_plotter = CameraPlotter(w_, h_, cam_pos, cam_lookat, cam_up, cam_right, fov)
    raycaster = RayCaster(w_, h_, cam_pos, cam_lookat, cam_up, cam_right, fov)
    ray_plotter = RayPlotter()

    pos_cube_center = cfg["pos_cube_center"]
    mint, maxt = cfg["mint"], cfg["maxt"]
    rgba = 'rgba(0,0,0,0.2)'
    cube_plotter = CubePlotter(pos_cube_center, mint, maxt, rgba)
    trace_cube = cube_plotter.get_trace_cube()
    cam_plotter.fig.add_trace(trace_cube)

    grid_resolution = cfg["grid_resolution"]    
    aabb = AABB(pos_cube_center, mint, maxt, grid_resolution)

    thres_pos = 0.8#1.0#0.5#0.65
    assert thres_pos > 0, "thres_pos should be positive"
    high_density_val = 0.95
    min_density_val, max_density_val = 0.2, 0.95#0.1, 0.2
    density_mode = cfg["density_mode"]
    aabb.create_density_volume(thres_pos, min_density_val, max_density_val, high_density_val, density_mode)

    
    num_cam = cfg["num_cam"]
    for i in range(num_cam):

        cam_mover = CameraMover()        
        step_x, step_y, step_z = cfg["cam_step"]
        rotate_x, rotate_y, rotate_z = cfg["cam_rotate"]    
        if cfg["cam_rotate_around_aim_flag"]:
            # M_ext = cam_mover.M_ext
            # _, cam_pos_world, cam_lookat_world, _, _ = cam_plotter.cam2world(M_ext)
            cam_pos_world = np.array([step_x,step_y,step_z])
            aim_pos_world = np.array(cfg["aim_pos_world"])
            # cam_mover.rotate_aim_pos_world(cam_pos_world, cam_lookat_world, aim_pos_world)
            cam_mover.rotate_aim_pos_world(cam_pos_world, cam_lookat, aim_pos_world)
        else:
            cam_mover.step_x(step_x)
            cam_mover.step_y(step_y)
            cam_mover.step_z(step_z)
            cam_mover.rotate_x(rotate_x)
            cam_mover.rotate_y(rotate_y)
            cam_mover.rotate_z(rotate_z)

        M_ext = cam_mover.M_ext
        cam_plotter.add_cam(M_ext, plot_color='blue', plot_name='cam'+str(i))
        cam_plotter.add_trace_cam_coord()
        # cam_plotter.add_trace_cam_screen()

        ray_dirs_world, cam_pos_world = raycaster.raycast(M_ext)
        logger.debug(f"ray_dirs_world.shape: {ray_dirs_world.shape}, cam_pos_world: {cam_pos_world}")

        # ray cast
        tmin_0 = 4.5#3.5
        tmax_0 = 9.5#4.5
        tmin = tmin_0
        tmax = tmax_0
        assert tmin < tmax, "ray marching range"
        trace_ray_lis = []
        trace_sampling_point_lis = []

        num_points = cfg["num_points"]
        tmax_margin = 2.0


        light_pos = np.array([0.,0.,0.])
        render_img_from_volume(ray_dirs_world, w_, h_, num_points, aabb, cam_pos_world, light_pos)


        ray_plot_flag = cfg["ray_plot_flag"]
        plot_thres_reso = cfg["ray_plot_thres_reso"]
        if w_ >= plot_thres_reso:
            ray_plot_flag = False
            logger.info(f"not plot ray. beacause w_ is over threshold: {w_} >= {plot_thres_reso}")
        if not ray_plot_flag:
            continue

        for px in range(w_):
            for py in range(h_):
            # for ray_idx in ray_idx_plot_lis:
                ray_dir_world = ray_dirs_world[px][py]
                tmin, tmax, if_intersect = aabb.ray_intersect_and_get_mint(ray_dir_world, cam_pos_world)
                if not if_intersect:
                    tmin = tmin_0
                    tmax = tmax_0
                assert tmin < tmax, "ray marching range"
                ray_idx = px*w_+py
                logger.info(f"ray idx: {ray_idx}/{w_*h_}")
                traces_ray = ray_plotter.get_traces_ray(cam_pos_world, ray_dir_world, tmax, tmax_margin, if_intersect, plot_name= "ray"+str(ray_idx))
                cam_plotter.fig.add_traces(traces_ray)

                if if_intersect:
                    dt = (tmax-tmin)/num_points
                    # r(t) = o + td = o + (tmin+dt*i)d
                    sampling_points = np.array([cam_pos_world + (tmin + dt*i) * ray_dir_world for i in range(num_points)])
                    trace_sampling_points = ray_plotter.get_trace_sampling_point(sampling_points)
                    cam_plotter.fig.add_trace(trace_sampling_points)
        
    if cfg["plot_flag"]:
        cam_plotter.fig_show()
