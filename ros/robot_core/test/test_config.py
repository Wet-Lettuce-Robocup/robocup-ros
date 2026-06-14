# Robot Core
# Copyright (C) 2026  Dry Lettuce
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os

import pytest
import yaml

YAML_FILE_PATH = os.path.join(
    os.path.dirname(__file__),
    '..',
    'config',
    'params.yaml',
)


def test_yaml_syntax():
    """Test if the YAML file has valid syntax."""
    assert os.path.exists(YAML_FILE_PATH), f'Config file not found at {YAML_FILE_PATH}'

    try:
        with open(YAML_FILE_PATH, 'r') as f:
            config_data = yaml.safe_load(f)
    except yaml.YAMLError as e:
        pytest.fail(f'YAML syntax is invalid: {e}')

    assert config_data is not None, 'YAML file is empty'
