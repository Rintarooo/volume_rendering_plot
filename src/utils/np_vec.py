import numpy as np
from src.utils.logger_global import logger

def degree2radian(deg):
    return deg * np.pi / 180.

def normalize(x):
    # logger.debug(f"x:{x}, np.any(x):{np.any(x)}")
    if np.any(x):
        try:
            return x / np.linalg.norm(x)
        except ZeroDivisionError as e:
            logger.debug(f"normalize x: {x}, norm: {np.linalg.norm(x)}, zero devide error: {e}")
    else:# array elements are all zero
        return x
