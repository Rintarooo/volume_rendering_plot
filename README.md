# Volume Rendering Visualization in 3D

* Ray Casting
* Volume Rendering
* AABB ray intersection

## Overview
I used `plotly` for interactive 3D visualization.

### Plot

![スクリーンショット 2023-09-26 午後3 25 15](https://github.com/Rintarooo/volume_rendering_plot/assets/51239551/37e56d6e-6934-4a5a-a195-c63e4133fb0a)
![スクリーンショット 2023-09-26 午後3 26 22](https://github.com/Rintarooo/volume_rendering_plot/assets/51239551/a15098b5-37e4-4513-b0ee-40773c02ed43)
![スクリーンショット 2023-09-26 午後3 26 31](https://github.com/Rintarooo/volume_rendering_plot/assets/51239551/70e32dc5-9451-4bbe-acf4-9e6aff7f9760)

### Rendered Image
![render](https://github.com/Rintarooo/volume_rendering_plot/assets/51239551/cfea8558-9fd0-4cce-8912-0e5f8cb30a4f)

### density plot for a ray
![density_plot](https://github.com/Rintarooo/volume_rendering_plot/assets/51239551/15e9a172-15c0-44c4-9840-a63dc946be7a)


### Maya
![スクリーンショット](https://github.com/Rintarooo/volume_rendering_plot/assets/51239551/9a2a5974-7d53-44fe-b2b7-d0854ad14c36)


<!-- <img src="https://github.com/Rintarooo/volume_rendering_plot/assets/51239551/9a2a5974-7d53-44fe-b2b7-d0854ad14c36)" width="30"> -->
<!-- <img src="https://github.com/Rintarooo/volume_rendering_plot/assets/51239551/9a2a5974-7d53-44fe-b2b7-d0854ad14c36)"> -->

## Usage
### Execute Python
```bash
poetry run python -m main config/cfg.yaml
```

modify `src/utils/logger_global.py` and `config/cfg.yaml` as you like.


## Setup

* pyenv + poetry
    * python 3.10.1
    * plotly

### pyenv
```bash
# version of pyenv
pyenv --version

# version of python installed in pyenv
pyenv versions

# list v3.10 python in pyenv
pyenv install --list | grep -i "3.10."

# install python v3.10.1 in pyenv
pyenv install 3.10.1

# create '.python-version' file in the current directory
pyenv local 3.10.1

# confirm * points at v3.10.1
pyenv versions
```

### poetry
```bash
# create .venv directory in the current directory
poetry config virtualenvs.in-project true

# Enter all [y]
poetry init --python ">=3.9,<3.13"

# Make sure you correctly installed python, get in poetry shell
poetry shell
# in poetry shell, this command should output 'Python 3.10.1'
python -V

# add packages
poetry add numpy matplotlib plotly scipy pyyaml
```

## Reference
* https://github.com/shauseth/pyramidify