# Plot volume rendering
## Usage

## Setup

* pyenv + poetry
    * python 3.10.1

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

# get in poetry shell
poetry shell

# in poetry shell, this command should output 'Python 3.10.1'
python -V

# add packages
poetry add numpy matplotlib plotly scipy
```

## Execute Python
```bash
# poetry run python main.py
poetry run python -m main
```
