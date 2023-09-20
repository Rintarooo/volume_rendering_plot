import logging
import os

"""
# https://yassanabc.com/2021/04/14/%E3%80%90python%E3%80%91logging%E3%81%AE%E6%AD%A3%E3%81%97%E3%81%84%E4%BD%BF%E3%81%84%E6%96%B9/
# https://note.com/enkey/n/na366b382800a
# https://stackoverflow.com/questions/533048/how-to-log-source-file-name-and-line-number-in-python
"""

logger = logging.getLogger(__name__)
# logger = logging.getLogger()
# logging.disable(logging.CRITICAL)

log_file = './logs/logging.log'

if os.path.exists(log_file):
    os.remove(log_file)
    
filehandler = logging.FileHandler(log_file)
# streamhandler = logging.StreamHandler()

# format = '[%(asctime)s][%(levelname)s] %(message)s'
format = '[%(asctime)s][%(levelname)s] {%(module)s.%(funcName)s, l: %(lineno)d} %(message)s'
# datefmt='%Y/%m/%d %I:%M:%S'
datefmt='%I:%M:%Ss'

formatter = logging.Formatter(format, datefmt)
# streamhandler.setFormatter(formatter)
filehandler.setFormatter(formatter)
# logger.addHandler(streamhandler)
logger.addHandler(filehandler)

logger.setLevel(logging.DEBUG)  # DEBUG #INFO #WARNING

logger.info("Logger INFO mode Start!")
logger.debug("Logger DEBUG mode Start!")



# class LoggerGlobal:
#     def __init__(self) -> None:

# def get_logger(log_file = './out/logging.log'):
    # return logger

# if __name__ == "__main__":
#     global logger
#     log_file = './out/logging.log'
#     logger = get_logger(log_file)
#     logger.info("Logger INFO mode Start!")


    # lg = LoggerGlobal()