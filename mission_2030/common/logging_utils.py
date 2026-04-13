import logging

def setup_logger(name: str, log_file: str = "mission_log.txt", level=logging.INFO) -> logging.Logger:
    """Sets up a structured logger for mission events.
    
    Prevents duplicate handlers if called multiple times with the same name.
    """
    logger = logging.getLogger(name)
    
    # If this logger already has handlers, don't add more (prevents log duplication)
    if logger.handlers:
        return logger
    
    logger.setLevel(level)

    formatter = logging.Formatter(
        '[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s',
        datefmt='%H:%M:%S'
    )

    # Console Handler
    ch = logging.StreamHandler()
    ch.setLevel(level)
    ch.setFormatter(formatter)
    
    # File Handler
    fh = logging.FileHandler(log_file)
    fh.setLevel(level)
    fh.setFormatter(formatter)

    logger.addHandler(ch)
    logger.addHandler(fh)
    
    return logger
