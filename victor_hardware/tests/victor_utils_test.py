#!/usr/bin/env python

import pytest
from victor_hardware import victor_utils as vu


def test_jvq_conversions(self):
    q = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    jvq = vu.list_to_jvq(q)
    assert jvq.joint_1 == 1.0
    assert jvq.joint_7 == 7.0

    and_back = vu.jvq_to_list(jvq)
    assert len(and_back) == 7

    for i in range(len(q)):
        assert q[i] == and_back[i]

    vu.list_to_jvq([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])