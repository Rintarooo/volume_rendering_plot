import numpy as np
import plotly.graph_objects as go

from src.utils.logger_global import logger
from src.utils.camera_plotter import CameraPlotter
from src.utils.camera_mover import CameraMover


if __name__ == "__main__":
    w_, h_, plot_scale = 3, 3, 1.0
    fig = go.Figure()
    plotter = CameraPlotter(fig, w_, h_, plot_scale)
    mover = CameraMover()
    num_camera = 10
    for i in range(num_camera):
        camera_mover = CameraMover()
        radius = 5
        degree = 360/num_camera * i
        angle = np.radians(degree)
        step_x = radius*np.cos(angle)
        step_z = radius*np.sin(angle)
        camera_mover.step_x(step_x)
        camera_mover.step_z(step_z)
                
        # camera_mover.rotate_y(degree)
        # camera_mover.rotate_x(-360./num_camera * i - 90)
        camera_mover.rotate_y(-360./num_camera * i - 90)
        M_ext = camera_mover.M_ext

        # M_ext = np.array(frame['transform_matrix'])
        plotter.add_cam(M_ext, plot_color='blue', plot_name='frame'+str(i))
        # plotter.add_camera_coord(M_ext)
        # plotter.add_rays(M_ext)

    plotter.add_trace_cam_coord()
    plotter.add_trace_cam_screen()
    plotter.fig_show()