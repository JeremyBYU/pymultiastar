import logging
logger = logging.getLogger('foo')
logger.addHandler(logging.NullHandler())