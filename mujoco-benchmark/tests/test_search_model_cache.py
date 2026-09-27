import hashlib
import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_model_cache import attach,restore


def setup(tmp):
    source=tmp/'original.mjb';source.write_bytes(b'immutable model')
    run=tmp/'run';(run/'replay_assets').mkdir(parents=True)
    link=run/'replay_assets/model.mjb';link.symlink_to(source)
    return source,run,link,hashlib.sha256(source.read_bytes()).hexdigest()


def test_cache_identical_and_durable_link_restored(tmp_path):
    source,run,link,h=setup(tmp_path)
    attach(run,source,h,tmp_path/'cache')
    assert link.resolve()!=source and link.read_bytes()==source.read_bytes()
    restore(run)
    assert link.resolve()==source
    assert json.loads(link.with_name('model_cache.json').read_text())['restored']


def test_corrupt_existing_cache_is_rejected_not_overwritten(tmp_path):
    source,run,link,h=setup(tmp_path)
    cache=tmp_path/'cache';cache.mkdir();(cache/(h+'.mjb')).write_bytes(b'bad')
    with pytest.raises(ValueError,match='Corrupt'):attach(run,source,h,cache)
    assert link.resolve()==source and (cache/(h+'.mjb')).read_bytes()==b'bad'


def test_changed_source_does_not_get_cached_or_replace_link(tmp_path):
    source,run,link,h=setup(tmp_path);source.write_bytes(b'changed')
    with pytest.raises(ValueError,match='differs'):attach(run,source,h,tmp_path/'cache')
    assert link.resolve()==source and not list((tmp_path/'cache').glob('*.mjb'))
