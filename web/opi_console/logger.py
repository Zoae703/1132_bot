"""
Structured logging for the Orange Pi console.

Provides coloured console output and rotating file logs.
Distinguishes between debug telemetry and important events.
"""

import logging
import logging.handlers
import sys
from pathlib import Path
from typing import Optional


class ColoredFormatter(logging.Formatter):
    """Coloured console output."""
    COLORS = {
        "DEBUG": "\033[36m",     # cyan
        "INFO": "\033[32m",      # green
        "WARNING": "\033[33m",   # yellow
        "ERROR": "\033[31m",     # red
        "CRITICAL": "\033[41m",  # red background
    }
    RESET = "\033[0m"

    def format(self, record: logging.LogRecord) -> str:
        color = self.COLORS.get(record.levelname, "")
        msg = super().format(record)
        return f"{color}{msg}{self.RESET}"


def setup_logging(
    log_level: str = "INFO",
    log_file: Optional[str] = None,
    event_file: Optional[str] = None,
    console: bool = True,
    max_bytes: int = 10 * 1024 * 1024,
    backup_count: int = 5,
) -> logging.Logger:
    """Configure the root logger for the opi_console application.

    Args:
        log_level: One of DEBUG, INFO, WARNING, ERROR, CRITICAL.
        log_file: Path to the main log file (rotating, DEBUG+).
        event_file: Path to the event log (key events only, INFO+).
        console: Enable coloured console output.

    Returns:
        The root logger.
    """
    logger = logging.getLogger("opi_console")
    logger.setLevel(getattr(logging, log_level.upper(), logging.INFO))
    for handler in logger.handlers:
        handler.close()
    logger.handlers.clear()

    # Console handler
    if console:
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(getattr(logging, log_level.upper(), logging.INFO))
        ch.setFormatter(ColoredFormatter(
            "%(asctime)s [%(levelname)-8s] %(message)s",
            datefmt="%H:%M:%S",
        ))
        logger.addHandler(ch)

    # File handler (DEBUG level, rotated)
    if log_file:
        try:
            Path(log_file).parent.mkdir(parents=True, exist_ok=True)
            fh = logging.handlers.RotatingFileHandler(
                log_file, maxBytes=max_bytes, backupCount=backup_count,
            )
            fh.setLevel(logging.DEBUG)
            fh.setFormatter(logging.Formatter(
                "%(asctime)s [%(levelname)-8s] %(name)s: %(message)s",
            ))
            logger.addHandler(fh)
        except OSError as e:
            logger.warning("File logging disabled for %s: %s", log_file, e)

    # Event file handler (INFO level, key events only)
    if event_file:
        try:
            Path(event_file).parent.mkdir(parents=True, exist_ok=True)
            eh = logging.handlers.RotatingFileHandler(
                event_file, maxBytes=max_bytes, backupCount=backup_count,
            )
            eh.setLevel(logging.INFO)
            eh.setFormatter(logging.Formatter(
                "%(asctime)s EVENT %(message)s",
            ))
            logger.addHandler(eh)
        except OSError as e:
            logger.warning("Event logging disabled for %s: %s", event_file, e)

    return logger


def get_logger(name: str = "") -> logging.Logger:
    """Get a child logger of the opi_console root logger."""
    if name:
        return logging.getLogger(f"opi_console.{name}")
    return logging.getLogger("opi_console")
