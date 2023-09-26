import yaml
import sys
from src.utils.logger_global import logger

class YamlLoader():
    def __init__(self, yaml_file) -> None:
        self.yaml_file = yaml_file

    def load_yaml(self):
        # https://maku77.github.io/python/io/yaml.html
        try:
            with open(self.yaml_file) as f:
                cfg = yaml.safe_load(f)
                logger.info(f"load YAML: {self.yaml_file}...\n{cfg}")
        except Exception as e:
            logger.info(f'Exception occurred while loading YAML: {self.yaml_file}')
            logger.info(f"error: {e}")
            sys.exit(1)
        return cfg
