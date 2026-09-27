"""A permitted search contact must remain visible in recorded-rollout captions."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from render_contact_diagnostic import authorized_force_caption


def test_caption_preserves_rachis_and_nonseating_pedicel_force():
    row = dict(contact_categories_private={
        'target_rachis_touch': {'force_sum_N': 2.1},
        'target_pedicel_touch': {'force_sum_N': .4},
        'target_pedicel_contact': {'force_sum_N': 0.},
        'forbidden_contact': {'force_sum_N': .02},
    }, target_displacement_m=.003, seated=False)
    caption = authorized_force_caption(row)
    assert 'rachis=2.100' in caption
    assert 'ped-seat/touch=0.000/0.400' in caption
    assert 'forbidden=0.020' in caption
    assert 'seated=False' in caption


def test_legacy_caption_has_zero_for_absent_search_categories():
    row = dict(contact_categories_private={}, target_displacement_m=0., seated=True)
    assert 'rachis=0.000' in authorized_force_caption(row)
