import logging
import os
from pathlib import Path

class Logger():
    
    def __init__(self, logger_name:str, log_path: Path, logger_filename: str):
        self.logger_name = logger_name
        if not os.path.exists(log_path):
            os.makedirs(log_path)
        self.logger = self.setup_logger(logger_name, Path(log_path) / logger_filename)

    def setup_logger(self, name, log_file, level=logging.DEBUG):
        """To setup as many loggers as you want"""
        formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        handler = logging.FileHandler(log_file)     
        handler.setFormatter(formatter)

        logger = logging.getLogger(name)
        logger.setLevel(level)
        logger.addHandler(handler)

        return logger
    
    def info(self, message):
        self.logger.info(message)
    
    def debug(self, message):
        self.logger.debug(message)

    def error(self, message):
        self.logger.error(message)