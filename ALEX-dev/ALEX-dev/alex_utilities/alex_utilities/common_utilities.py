from pathlib import Path
from functools import reduce, lru_cache
from ament_index_python.packages import get_package_prefix

@lru_cache()
def list_executables(package_name: str):
    """
    Lists executable files for a given package.
    """
    try:
        # Get the package installation prefix
        package_prefix = get_package_prefix(package_name)
    except ValueError as e:
        # Log a warning if the package isn't found
        print(f"Error: Package '{package_name}' not found. Details: {e}")
        return set()

    # Construct the path to the 'lib/<package_name>' directory
    bin_dir = Path(package_prefix) / 'lib' / package_name

    # Check if the directory exists
    if not bin_dir.is_dir():
        return set()

    try:
        # Find and return all executable files in the directory as a set
        return {
            file.name for file in bin_dir.iterdir()
            if file.is_file() and file.stat().st_mode & 0o111  # Check if file is executable
        }
    except Exception as e:
        # Log unexpected errors during file iteration or checks
        print(f"Unexpected error while accessing executables in '{bin_dir}': {e}")
        return set()

def change_case(str:str):
    """Converts the string to a snake case format
    Example Usage:
    from alex_utilities.common_utilities import change_case

    a = change_case("TestString")
    print(a)
    >> test_string

    """
    return reduce(lambda x, y: x + ('_' if y.isupper() else '') + y, str).lower()

def rosmsg_to_dict(msg):
    if isinstance(msg, list): 
        return [rosmsg_to_dict(item) for item in msg]
    elif hasattr(msg, '__slots__'):
        return {field.lstrip('_'): rosmsg_to_dict(getattr(msg, field)) for field in msg.__slots__}
    return msg

def get_device_namespace(device_name: str):
    return device_name.replace("-","_")
