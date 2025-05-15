from enum import Enum

LIFT_HEIGHT = 30

class ContainerType(Enum):
    ML_200_BOTTLE = "200ml Bottle"
    ML_500_VOLUMETRIC_FLASK = "500ml Volumetric Flask"
    ML_200_VOLUMETRIC_FLASK = "200ml Volumetric Flask"
    ML_1000_BEAKER = "1000ml beaker"
    ML_200_BEAKER = "200ml beaker"
    WEIGHTBOAT = "weightboat"

CONTAINER_DIMENTIONS = {
    ContainerType.ML_500_VOLUMETRIC_FLASK : dict(
        height = 275,
        width = 100,
    ),
    ContainerType.ML_200_VOLUMETRIC_FLASK : dict(
        height = 275,
        width = 100,
    ),
    ContainerType.ML_200_BOTTLE : dict(
        height = 180,
        width = 50,
    ),
    ContainerType.ML_1000_BEAKER : dict(
        height = 150,
        width = 115,
    ),
    ContainerType.ML_200_BEAKER : dict(
        height = 118,
        width = 85,
    ),
    ContainerType.WEIGHTBOAT : dict(
        height = 55,
        width  = 105,
    ),
}
