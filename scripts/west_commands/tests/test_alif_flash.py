# Copyright (c) 2026 East Agile
#
# SPDX-License-Identifier: Apache-2.0

import logging

import pytest

from runners import get_runner_cls
from runners.alif_flash import AlifImageBinaryRunner


def test_alif_flash_runner_is_registered():
    # Every boards/alif/*/board.cmake selects this runner, so it has to
    # register even when the optional 'fdt' package is not installed.
    assert get_runner_cls('alif_flash') is AlifImageBinaryRunner


def test_get_itcm_address_without_fdt(monkeypatch):
    monkeypatch.setattr('runners.alif_flash.MISSING_REQUIREMENTS', True)

    with pytest.raises(RuntimeError, match='fdt'):
        AlifImageBinaryRunner.get_itcm_address(logging.getLogger('test'))
