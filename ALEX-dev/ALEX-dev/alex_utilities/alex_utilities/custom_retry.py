from time import sleep
import functools
import inspect
from rclpy.impl.rcutils_logger import RcutilsLogger

class RetryException(Exception):
    """
    Custom exception to be raised when all retries fail.
    """
    def __init__(self, message="All retries failed", retries=0, attempts=0, func_name=None, errors: set[str] = None):
        exception_message = ""
        if func_name:
            exception_message += f"Function: {func_name} | "
        exception_message += f"Message: {message} | Retries: {retries}/{attempts}"
        if errors:
            exception_message += f" | Errors: {errors}"
        super().__init__(exception_message)

# ----------------------------
# Synchronous Retry Logic
# ----------------------------

def retry_function(
    func,
    retries=3,
    delay=1,
    backoff=2,
    exceptions=(Exception,),
    *args,
    **kwargs
):
    """
    Core retry logic using a while loop (sync version).
    """
    attempt = 0
    current_delay = delay

    while attempt < retries:
        try:
            return func(*args, **kwargs)
        except exceptions:
            attempt += 1
            if attempt == retries:
                raise RetryException(retries=retries, attempts=attempt)
            sleep(current_delay)
            current_delay *= backoff

def retry_decorator(retries=3, delay=1, backoff=2, exceptions=(Exception,)):
    """
    Decorator that retries a synchronous function using retry_function.
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            return retry_function(func, retries, delay, backoff, exceptions, *args, **kwargs)
        return wrapper
    return decorator

# ----------------------------
# Asynchronous Retry Logic
# ----------------------------

async def async_retry_function(func, *args, retries=3, delay=1, backoff=2, exceptions=(Exception,), logger: RcutilsLogger|None =None,  **kwargs):
    """
    Core retry logic for async functions using a while loop.

    Note:
        This is a **blocking call**. The thread will be paused until the event is triggered.
        Use with caution in single-threaded applications or in performance-critical paths.
    """

    if not logger:
        logger = lambda a: print(f"[{func.__name__}]: "+f"{a}")
        logger.info = lambda a: print(f"[{func.__name__}] [INFO]: "+f"{a}")
        logger.error = lambda a: print(f"[{func.__name__}] [ERROR]: "+f"{a}")
        logger.fatal = lambda a: print(f"[{func.__name__}] [FATAL]: "+f"{a}")

    attempt = 0
    current_delay = delay
    errors: set[str] = set()

    while attempt < retries:
        try:
            logger.info(f"Attempt {attempt + 1}/{retries} of {func.__name__}")
            result = await func(*args, **kwargs)
            logger.info(f"Operation {func.__name__} completed without raising error after {attempt} attempts.")
            return result
        except exceptions as error:
            attempt += 1
            errors.add(str(error))
            if attempt == retries:
                raise RetryException(message=f"All retries failed", retries=retries, attempts=attempt, func_name=func.__name__, errors=errors)
            logger.error(f"Attempt {attempt} failed with error: {error} | Retrying in {current_delay} seconds...")
            sleep(current_delay)
            current_delay *= backoff

def async_retry_decorator(retries=3, delay=1, backoff=2, exceptions=(Exception,)):
    """
    Decorator that retries an async function using async_retry_function.

    Note:
        This is a **blocking call**. The thread will be paused until the event is triggered.
        Use with caution in single-threaded applications or in performance-critical paths.
    """
    def decorator(func):
        if not inspect.iscoroutinefunction(func):
            raise TypeError("async_retry_decorator can only be used with async functions")

        @functools.wraps(func)
        async def wrapper(*args, **kwargs):
            return await async_retry_function(func, retries, delay, backoff, exceptions, *args, **kwargs)
        return wrapper
    return decorator
