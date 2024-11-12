import pytest
import numpy as np

from transformations import transform_wind_velocity

@pytest.mark.parametrize(
    "V_world, drone_rotation_deg, expected",
    [
        (10.0, 50.0, np.array([-6.4278761,  -7.66044443,  0.   ]))
    ],
)


def test__wind_transform_to_body_correct(
    V_world: float, drone_rotation_deg: float, expected: np.ndarray
):
    assert np.allclose(
        transform_wind_velocity(V_world, drone_rotation_deg), expected
    )
