import pytest
from wind_field import WindField

@pytest.fixture
def wind_field():
    # Create an instance of the WindField class for testing
    return WindField()

def test_set_wind_speed(wind_field):
    # Test the set_wind_speed method
    wind_field.set_wind_speed(10)
    assert wind_field.get_wind_speed() == 10

def test_get_wind_direction(wind_field):
    # Test the get_wind_direction method
    assert wind_field.get_wind_direction() == "North"

def test_set_wind_direction(wind_field):
    # Test the set_wind_direction method
    wind_field.set_wind_direction("South")
    assert wind_field.get_wind_direction() == "South"

def test_get_wind_field(wind_field):
    # Test the get_wind_field method
    wind_field.set_wind_speed(10)
    wind_field.set_wind_direction("East")
    assert wind_field.get_wind_field() == "Wind field data"

def test_update_wind_field(wind_field):
    # Test the update_wind_field method
    wind_field.update_wind_field()
    assert wind_field.get_wind_field() == "Updated wind field data"