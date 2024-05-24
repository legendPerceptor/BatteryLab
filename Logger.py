import logging
from pathlib import Path

class Logger():
    
    def __init__(self, device_name:str, log_path: Path):
        self.device_name = device_name
        self.logger = self.setup_logger(device_name, log_path)

    def setup_logger(self, name, log_file, level=logging.INFO):
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