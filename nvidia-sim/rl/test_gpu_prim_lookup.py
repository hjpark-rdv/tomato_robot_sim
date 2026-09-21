from gpu_prim_lookup import explicit_path_lookup


class Prim:
    def __init__(self,active=True,defined=True,loaded=True,abstract=False,proxy=False):
        self.flags=active,defined,loaded,abstract,proxy
    def IsActive(self):return self.flags[0]
    def IsDefined(self):return self.flags[1]
    def IsLoaded(self):return self.flags[2]
    def IsAbstract(self):return self.flags[3]
    def IsInstanceProxy(self):return self.flags[4]


class Stage:
    def __init__(self,prim):self.prim=prim;self.paths=[]
    def GetPrimAtPath(self,path):self.paths.append(path);return self.prim


def test_literal_lookup_skips_stage_scan_but_preserves_regex_fallback():
    prim=Prim();stage=Stage(prim);calls=[]
    def fallback(path,stage):calls.append(path);return 'regex'
    assert explicit_path_lookup('/World/envs/env_999/Robot',stage,fallback) is prim
    assert calls==[] and stage.paths==['/World/envs/env_999/Robot']
    for regex in ('/World/env_.*/Robot','/World/env_[0-9]+/Robot','relative','/World/*'):
        assert explicit_path_lookup(regex,stage,fallback)=='regex'
    assert len(stage.paths)==1


def test_literal_lookup_obeys_default_traversal_visibility():
    for prim in (None,Prim(active=False),Prim(defined=False),Prim(loaded=False),Prim(abstract=True),Prim(proxy=True)):
        assert explicit_path_lookup('/World/Robot',Stage(prim),lambda *_:1) is None


def test_multi_match_helper_preserves_get_all_children_predicate():
    from gpu_prim_lookup import explicit_path_matches
    for prim in (Prim(),Prim(active=False),Prim(defined=False),Prim(abstract=True)):
        assert explicit_path_matches('/World/Robot',Stage(prim),lambda *_:None)==[prim]
    assert explicit_path_matches('/World/Robot',Stage(Prim(proxy=True)),lambda *_:None)==[]
    assert explicit_path_matches('/World/env_.*/Robot',Stage(None),lambda *_:['fallback'])==['fallback']
