import numpy as np
from typing import Any

import matplotlib.pyplot as plt
from src.volume_rendering.aabb import AABB
from src.utils.logger_global import logger

def render_img_from_volume(
    ray_dirs: Any,
    w_: Any,
    h_: Any,
    num_points: Any,
    aabb: Any,
    cam_pos: Any,
    light_pos: Any,
) -> None:    
    """"
    absorptionCoef, scatteringCoef = 0.1, 0.1
    # 減衰係数(extinction coefficient) = 吸収係数(absorption coefficient)+散乱係数(scattering coefficient)
    # https://qiita.com/Renard_Renard/items/fd706def17cb8d1e8ec4
    # 減衰が発生し、その結果残る光の割合を透過率(transmittance)と呼ぶ
    # T = e^(-tau)
    # extinctionCoef = 0.1
    extinctionCoef = absorptionCoef + scatteringCoef
    """
    # point_pos = np.array([trace_sampling_point_lis[0][0]["x"][0], trace_sampling_point_lis[0][0]["y"][0], trace_sampling_point_lis[0][0]["z"][0]])
    # cur_density = aabb.read_volume(point_pos)

    
    # for each ray    
    channels = 3
    render_img = np.zeros([w_, h_, channels])
    
    ## for density plot
    density_plot_dic = {"density_lis":[], "tmin":0, "tmax":0, "ray_idx":0}
    density_plot_index_ray = int(len(ray_dirs)/2)#5
    density_plot_dic["ray_idx"] = density_plot_index_ray
    assert density_plot_index_ray < len(ray_dirs), "ray index is over list ray_dirs"

    # for ray_idx in range(len(ray_dirs)):
    cnt_intersect = 0
    for px in range(w_):
        # idx_px = px * w_
        for py in range(h_):
            # black
            # render_color = np.array([0,0,0])
            # background_color = np.array([0,0,0])
            render_color = np.array([0.,0.,0.])
            ## light blue
            background_color = np.array([157.,204.,224.])
            ## transmittance, initialize transmission to 1 (fully transparent, so background color is rendered)
            T_ = 1. 
            # ray_idx = idx_px + py
            # assert ray_idx < len(ray_dirs), "ray index is over list ray_dirs"
            # ray_dir = ray_dirs[ray_idx]
            ray_idx = py * w_ + px
            ray_dir = ray_dirs[px][py]
            tmin, tmax, if_intersect = aabb.ray_intersect_and_get_mint(ray_dir, cam_pos)
            logger.debug(f"[px,py]: [{px},{py}], if_intersect: {if_intersect}")
            if not if_intersect:
                tmin = 4.5
                tmax = 9.5
                # black if it does not intersect
                # render_color = np.array([0,0,0])
                render_color = background_color
            assert tmin < tmax, "ray marching range"
            delta = (tmax-tmin)/num_points

            logger.info(f"ray idx: {ray_idx}/{w_*h_}")

            if if_intersect:
                cnt_intersect += 1
                for i in range(num_points):
                    # point_pos = np.array([cam_pos + (tmin + dt*t) * ray_dirs[ray_idx] for t in range(num_points)])
                    point_pos = cam_pos + (tmin+delta*i) * ray_dir
                    cur_density = aabb.read_volume(point_pos)
                    if ray_idx == density_plot_index_ray:
                        density_plot_dic["density_lis"].append(cur_density)
                        if i == 0:
                            density_plot_dic["tmin"] = tmin
                            density_plot_dic["tmax"] = tmax
                    # transmittance *= np.exp(-cur_density * extinctionCoef * delta)
                    alpha_i = 1 - np.exp(-cur_density * delta)
                    T_ *= np.exp(-cur_density * delta)
                    

                    # T_i is the product of 1. - alpha
                    # T_i = sampling_light(light_pos, point_pos, delta, aabb)
                    # T_i = 1.

                    # white, volume cell has homogenious color
                    color = np.array([255,255,255])#np.array([0,0,255])

                    # light emission
                    # weight_i = T_i*alpha_i
                    weight_i = T_*alpha_i

                    logger.debug(f"ray_idx: {ray_idx}, weight_i: {weight_i}, delta: {delta}, ray step: {i}, point_pos: {point_pos}, cur_density: {cur_density}")
                    # render_color += np.uint8(weight_i*color)
                    render_color += weight_i*color
                    # render_color += weight_i*color*5
                
            # logger.debug(f"ray_idx: {ray_idx}, px: {px}, py: {py}, np.uint8(render_color): {np.uint8(render_color)}")
            # logger.debug(f"img_x: {w_-px-1}, img_y: {h_-py-1}")
            # render_img[py][px] = np.uint8(render_color)
            # logger.debug(f"transmittance: {T_}, background_color: {background_color}, render_color: {render_color}")
            render_color =  T_ * background_color + (1.0-T_) * render_color
            # render_img[w_-px-1][h_-py-1] = np.uint8(render_color)
            render_img[px][py] = np.uint8(render_color)
            

    # plt.imsave("./render.png", render_img)
    render_path , density_plot_path = "./out/render.png", "./out/density_plot.png"
    plt.imsave(render_path, np.uint8(render_img))
    logger.info(f"save render_path: {render_path}")
    logger.debug(f"cnt_intersect: {cnt_intersect}/{len(ray_dirs)}")
    
    logger.debug(f"density_lis: {density_plot_dic['density_lis']}")
    plt.plot(np.linspace(density_plot_dic["tmin"], density_plot_dic["tmax"], len(density_plot_dic["density_lis"])), density_plot_dic["density_lis"], ".", markersize=8)
    plt.xlabel(f"tmin:{density_plot_dic['tmin']} - tmax:{density_plot_dic['tmax']}")
    plt.ylabel("density")
    plt.ylim(0, 1)
    plt.title(f"ray index: {density_plot_dic['ray_idx']}")
    logger.info(f"save density_plot_path: {density_plot_path}")
    plt.savefig(density_plot_path)