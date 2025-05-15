from enum import Enum

class SerialBand(Enum):
    """These enums are used in the serial input to identify the source device or data origin.
    
    Example: 

    LL:<data>

    1. LL:8
    2. LD:*OK
    
    """
    LIQUID_LEVEL = "LL:"
    LIQUID_DOSING = "LD:"
    SONICATOR_TEMPERATURE = "ST:"
    DC_ACTUATOR = "DA:"
    STEPPER_ACTUATOR = "SA:"
    ERROR = "ER:"
