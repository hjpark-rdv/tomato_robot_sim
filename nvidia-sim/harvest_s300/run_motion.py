#!/usr/bin/env python3
"""Bounded native Isaac 6.1 actuation/elastic/contact trial, with physical state log."""
import argparse
import json
import time
import traceback
import shutil
import hashlib
import math
import os
import sys
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('assembly',type=Path);p.add_argument('--output',type=Path,required=True)
p.add_argument('--mode',choices=['idle','jaw','pulse','belt_control','harvest'],default='harvest')
p.add_argument('--pulse-force-n',type=float,default=.1,help='Diagnostic world +X force, 1..2 seconds, then release')
p.add_argument('--pulse-body',help='One-world pulse fixture: exact rigid-body path; default remains the target fruit')
p.add_argument('--verify-leaf-contact-after-step',action='store_true',help='Opt-in: retain native logs; check solved leaf/tool or leaf/background-leaf overlap against the configured contact limit using native cooked convex hulls; unavailable geometry still stops')
p.add_argument('--verify-harvest-contact-after-step',action='store_true',help='Opt-in configured contact-limit native cooked solved-pose check for exact current-truss fruit/tool pairs; callback values retained')
p.add_argument('--verify-background-leaf-enclosures',action='store_true',help='Opt-in conservative closed-component hull bound for authored static leaf triangles; configured contact-limit solved-pose limit, native triangles remain unchanged')
p.add_argument('--capture-leaf-colliders',nargs='+',help='Exact existing compliant leaf/capsule paths for native cooked shape evidence before first motion step; observation only, never expands contact authorization')
p.add_argument('--capture-tool-colliders',nargs='+',help='Read-only exact existing tool Mesh native cooking capture before first trial step; all returned pieces retained')
p.add_argument('--seconds',type=float,default=0.)
p.add_argument('--condition-foliage-seconds',type=float,default=0.,help='Explicit idle initialization experiment, at most 3 seconds of recorded invalid exact foliage contacts followed by at least 1 second with the unchanged strict physics guard; never harvest credit')
p.add_argument('--headless',action='store_true');p.add_argument('--hz',type=int,default=240)
p.add_argument('--physics-only',action='store_true',help='Headless worker: official minimal native bootstrap without RTX renderer; CPU/CUDA equivalence must be validated for the selected model')
p.add_argument('--native-materialized-stage',action='store_true',help='Opt-in open the already verified full materialized USD dependency directly; record its exact SHA and compare resolved stage source rather than compiling a runtime override wrapper')
p.add_argument('--device',choices=['cpu','cuda:0'],default='cpu')
p.add_argument('--broadphase',choices=['auto','MBP','SAP','GPU'],default='auto',help='Independent broadphase diagnostic; auto retains CPU MBP / GPU defaults; solver device remains unchanged')
p.add_argument('--preparation',choices=['direct','fold_then_turn','low_turn_then_lift','tight_fold_then_turn','tight_fold_open_approach','tight_fold_level_approach','horizontal_ready'],default='fold_then_turn')
p.add_argument('--approach-pitch-deg',type=float,default=0.,help='Fixed entry pitch; 0 keeps CAD +Y horizontal and CAD +Z up. Never relaxed automatically')
p.add_argument('--approach-yaw-offset-deg',type=float,default=0.,help='Explicit horizontal entry direction offset about world +Z, relative to robot-to-target direction')
p.add_argument('--approach-yaw-mode',choices=['robot-radial','truss-axis'],default='robot-radial',help='truss-axis fits attached-fruit XY direction; horizontal yaw only, no extra radial offset')
p.add_argument('--truss-axis-min-fruits',type=int,default=2,help='Opt-in retain the last verified longitudinal yaw when fewer attached fruit remain; default historical two-fruit fit')
p.add_argument('--truss-entry-yaw-offset-deg',type=float,default=0.,help='Explicit +/-90 degree horizontal deviation from truss direction to avoid the own stem; full path checks still required')
p.add_argument('--sparse-entry-duration-scale',type=float,default=1.,help='Opt-in slower approach/withdraw for a sparse remainder retaining its longitudinal yaw; preserves all waypoint positions/rotations and physical properties')
p.add_argument('--entry-only',action='store_true',help='Horizontal-ready approach/hold/return demonstration, with open jaws; not a harvest attempt')
p.add_argument('--harvest-profile',choices=['legacy','belt_spin_release','belt_open_close_cycle'],default='legacy',help='Opt-in belt_spin_release or continuous belt_open_close_cycle; native breakage only')
p.add_argument('--belt-speed-m-s',type=float,default=.15,help='Differential belt surface speed, not motor rpm; zero with positive --belt-infeed-speed-m-s moves both contact surfaces inward equally')
p.add_argument('--belt-seconds',type=float,default=2.,help='Belt dwell after closing; cycle profile keeps belts running through both closes and opening')
p.add_argument('--close-seconds',type=float,default=3.,help='Closing duration for belt_spin_release; CAD jaw limits unchanged')
p.add_argument('--grasp-gap-mm',type=float,default=28.,help='Commanded mechanical jaw gap while grasping, within existing 28..96 mm travel; not collision margin or fruit compression')
p.add_argument('--grasp-depth-mm',type=float,default=65.,help='Aim depth behind the existing gripper nose; moves actual arm waypoints, never fruit poses')
p.add_argument('--entry-retract-mm',type=float,default=100.,help='Opt-in change level staging distance before the unchanged final belt entry; full joint/environment audits still required')
p.add_argument('--later-grasp-depth-mm',type=float,help='Opt-in absolute grasp depth after a completed native whole-truss cycle; first fresh cycle retains global depth and every later path uses existing IK/full collision checks')
p.add_argument('--grasp-height-offset-mm',type=float,default=0.,help='Explicit world-Z offset from the saved grasp aim; replan actual arm waypoints, preserve fruit and entry pitch')
p.add_argument('--last-fruit-height-offset-mm',type=float,help='Opt-in absolute robot aim height only when the selected exact truss has one native-attached fruit left; existing full IK/FCL/physics checks remain')
p.add_argument('--last-fruit-depth-mm',type=float,help='Opt-in absolute existing entry depth for that one remaining fruit; no reset or detachment credit')
p.add_argument('--last-fruit-grasp-sway-mm',type=float,default=0.,help='Opt-in robot CAD-X sway up to1.5mm only during the existing last-fruit close/reclose phases; keep yaw/pitch, jaw/belts/durations and native break rules')
p.add_argument('--grasp-lateral-offset-mm',type=float,default=0.,help='Explicit offset along the planned tool CAD +X axis; replan the existing horizontal belt entry, preserve fruit poses')
p.add_argument('--belt-ramp-seconds',type=float,default=0.,help='Smooth acceleration/deceleration within belt-spin duration; default preserves step command')
p.add_argument('--belt-infeed-speed-m-s',type=float,default=0.,help='Optional common inward belt speed during the same spin phase; no arm motion or external force')
p.add_argument('--belt-off-control',action='store_true',help='Belt-spin diagnostic: identical robot/jaw timeline, zero belt surface speeds')
p.add_argument('--whole-truss',action='store_true',help='Repeat the existing belt cycle on measured remaining fruit in one native scene; no reset')
p.add_argument('--harvest-group-policy',type=Path,help='SIM-GT ripe-group policy JSON: count only eligible fruit in the existing belt footprint; stop below its minimum; legacy selection remains available without this flag')
p.add_argument('--released-fruit-ground-diagnostic','--released-fruit-collection-diagnostic',dest='released_fruit_ground_diagnostic',action='store_true',help='Continue active harvesting after exact native-detached fruit hits verified floor or passive S300 collection deck; preserve raw impact violations as unverified collection diagnostics')
p.add_argument('--released-fruit-gripper-support-diagnostic',action='store_true',help='Record native-detached fruit impacts on exact gripper lower decks and carriage saddles without stopping; attached fruit and other pairs retain their contact limits')
p.add_argument('--released-fruit-same-truss-diagnostic',action='store_true',help='After verified native release, record own-truss fruit/branch impacts without stopping; keep physical collisions and other-truss/main-stem/robot guards')
p.add_argument('--rail-to-y',type=float,help='World-Y rail station: actuate the derived physical base joint before harvest; idle mode tests only rail travel')
p.add_argument('--rail-speed-m-s',type=float,default=.08,help='Smoothstep peak command rail speed, within (0,.2]m/s')
p.add_argument('--verify-rail-environment-preflight',action='store_true',help='Opt-in full frozen source/checkpoint farm collision screen of the initial physical rail_move at120Hz, before native initialization; native contact guards remain separate')
p.add_argument('--verify-preparation-environment-preflight',action='store_true',help='Opt-in120Hz farm screen of every initial phase before approach, including physically static leaf obstacles; native contact guards remain separate')
p.add_argument('--verify-full-environment-preflight',action='store_true',help='Opt-in full nominal farm collision check including approach/grasp/withdrawal; current truss tool contact only, actual physics guards retained')
p.add_argument('--horizontal-ready-normalize-turns',action='store_true',help='Opt-in nearest admissible2pi-equivalent staging joint angles before fixed-pose approach IK; same FK endpoint, exact reverse return, existing collision audits required')
p.add_argument('--diagnostic-motion-plan',type=Path,help='Import one independently audited OMPL preparation/reverse route into the existing initial-cycle runner; requires exact fresh model and one-cycle budget')
p.add_argument('--native-tool-collision-cache',type=Path,help='Opt-in exact existing native cooked tool pieces for offline self-collision audits; verified source hashes and all pieces are required, native physics stays unchanged')
p.add_argument('--max-truss-cycles',type=int,default=12,help='Finite plan/attempt budget for whole-truss mode, including rejected plans')
p.add_argument('--max-fruit-attempts',type=int,default=2,help='Finite attempts per attached fruit, within 1..8; explicit 8 allows two new robot-motion trials after a verified six-attempt parent, without erasing its recorded counts')
p.add_argument('--single-fruit-when-remaining',type=int,default=0,help='Opt-in individual centering when this exact truss has at most 1..3 attached fruits; zero preserves paired selection')
p.add_argument('--split-unproductive-pairs',action='store_true',help='Opt-in center fruits individually after a completed paired physical cycle leaves both native fruit joints attached; finite attempt limits remain unchanged')
p.add_argument('--truss-order',choices=['lowest-first','highest-first'],default='lowest-first',help='Explicit remaining-fruit ordering; highest-first also changes the initial grasp group')
p.add_argument('--preparation-height-first',action='store_true',help='At the existing ready XY, change height first, then move horizontally to staging; preserve entry orientation')
p.add_argument('--preparation-opening',choices=['ready','legacy-staging'],default='ready',help='Open fully at stationary ready pose before any approach motion; legacy-staging only reproduces old narrow-transfer diagnostics')
p.add_argument('--preparation-gap-mm',type=float,help='Legacy narrow transfer gap; used only with --preparation-opening legacy-staging. Default ready opening keeps transfer fully open at96mm')
p.add_argument('--ready-lift-offset-mm',type=float,default=0.,help='Opt-in continuous lift/arm redistribution at the unchanged ready TCP; fixed lift branch thereafter')
p.add_argument('--max-aim-translation-m',type=float,default=.5,help='Explicit bound on SIM-GT grasp translation; default historical 0.5m, row targeting may use up to2m')
p.add_argument('--near-trusses-metadata',type=Path,help='Whole-truss mode: harvest every aisle-facing truss on the active near-side stem using the existing farm inventory')
p.add_argument('--initial-truss-first',action='store_true',help='Near-truss mode: start with the specifically prechecked target truss, then consider every remaining eligible truss in the same measured scene')
p.add_argument('--save-route-checkpoint',action='store_true',help='Save exact final native body and articulation positions/velocities and joint-break inventory for immutable rail-route continuation')
p.add_argument('--checkpoint-cycle-states',action='store_true',help='Save measured native route state at each completed cycle boundary before planning the next approach')
p.add_argument('--resume-native-checkpoint',type=Path,help='Opt-in restart from a validated completed native cycle; restore actual q/qvel and native-detached inventory, without PhysX solver-cache continuity')
p.add_argument('--allow-resume-rail-travel',action='store_true',help='Explicitly allow physical world-Y rail travel from the validated same-plant checkpoint; preserve measured plant/robot state and prior drive targets, then audit the complete travel/harvest trace')
p.add_argument('--allow-resume-conditioned-start',action='store_true',help='Explicitly resume a separately proven canonical conditioned idle checkpoint; never treat an ordinary idle or failed trial as a completed cycle')
p.add_argument('--measured-ready-start',action='store_true',help='Use a checked real waypoint connector from measured horizontal-ready q; preserves actual initial position instead of assuming the authored reference height')
p.add_argument('--horizontal-ready-connector',choices=['cartesian','joint'],default='cartesian',help='Opt-in use a globally solved staging IK branch and audited joint preparation; final insertion remains level at the requested yaw')
p.add_argument('--horizontal-ready-seed-index',type=int,choices=range(9),help='Opt-in choose an exact existing staging IK seed for joint preparation; every resulting command still needs full collision audit')
p.add_argument('--allow-resume-leaf-proxy-revision',action='store_true',help='Resume only a proven immutable leaf approximation revision from the exact checkpoint source; all body/DOF/non-approximation properties must agree')
p.add_argument('--allow-resume-accuracy-proxy-revision',action='store_true',help='Resume only an explicitly audited convex cooking accuracy revision from the exact checkpoint source; authored shapes, physics properties, and state identity must agree')
p.add_argument('--allow-resume-attachment-transfer',action='store_true',help='Explicitly resume a proven transfer of the existing approved fruit-neck hypothesis to still-attached eligible near-side truss joints; no reset or unrelated property changes')
p.add_argument('--preparation-prefix-plan',type=Path,help='Reuse the exact first two tight-fold phases from a same-model saved plan; replan only the remaining preparation and entry')
p.add_argument('--isolate-background',action='store_true',help='Component fixture only: disable background plant contacts; never a farm success')
p.add_argument('--performance',choices=['baseline','optimized'],default='optimized',help='Same physical model; control USD/display/callback overhead')
p.add_argument('--threads',type=int,default=16,help='Explicit native PhysX CPU dispatcher threads; 0 keeps installed setting for historical diagnostics')
p.add_argument('--display-fps',type=float,default=30.,help='Wall-clock display cap; physics timestep is unchanged')
p.add_argument('--realtime',action='store_true',help='Cap completed simulated time to 1x wall time by sleeping; never skips physics steps or changes dt')
p.add_argument('--num-envs',type=int,default=1,help='Identical full dynamic copies, sharing immutable farm obstacles')
p.add_argument('--replicate',action='store_true',help='Use replicated namespace also for the one-world control')
p.add_argument('--replication-source-copy',choices=['flatten','reference'],default='flatten',help='Diagnostic USD composition variant; native equivalence must pass before scaling')
p.add_argument('--replication-update-boundary',action='store_true',help='Flush pending Kit USD notices once after copying roots, before native initialization')
p.add_argument('--replication-persist-layer-stack',action='store_true',help='Bounded clone diagnostic: export composed opinions to a new USDC and reopen before native initialization, without carrying GPU Cloner callbacks across stages')
p.add_argument('--replication-defer-gpu-bootstrap',action='store_true',help='Cold CUDA clone control: retain CPU/MBP and Fabric off through stage reopen, then register the requested GPU scene')
p.add_argument('--contact-backend',choices=['callback','tensor'],default='callback',help='Tensor sensor path needs actor-level/contact equivalence validation')
p.add_argument('--buffer-contact-events',action='store_true',help='Preserve per-step native guard; format raw callback contact evidence after measured loop')
p.add_argument('--native-contact-vector-copy',action='store_true',help='Opt-in lossless native Float3 slice copy for buffered callbacks; keeps every contact and per-step guard')
p.add_argument('--record-contact-frames',action='store_true',help='Buffered diagnostic: save native world contact positions/normals for actual collider support inspection')
p.add_argument('--verify-structural-scene-query',action='store_true',help='Opt-in native post-fetch enclosing-leaf OBB query for static triangle obstacles; complete positive/negative and pose controls are required for clear-only certification')
p.add_argument('--collision-filter',choices=['pairs','groups'],default='pairs',help='Groups compress the identical source exclusion graph; native A/B must pass before scaling')
p.add_argument('--position-iterations',type=int,default=64,help='Explicit solver convergence variant; validate contact/load/recovery before adopting')
p.add_argument('--velocity-iterations',type=int,default=4)
p.add_argument('--gpu-partitions',type=int,default=1,help='Native articulation GPU partitions; recorded explicitly for correctness/scaling checks')
p.add_argument('--gpu-pair-capacity',type=int,default=16384,help='Found/lost and total aggregate-pair allocation only; raise on native capacity errors and revalidate contacts')
p.add_argument('--solver',choices=['PGS','TGS'],default='PGS',help='Controlled native solver comparison; validate before adopting')
p.add_argument('--scene-ccd',action='store_true',help='CPU diagnostic: enable swept CCD at scene level as well as the existing rigid-body flags')
p.add_argument('--contact-limit-mm',type=float,default=1.,help='Common active-contact and initial-state penetration stop threshold (0.5..1mm); diagnostic tolerance only, no physics changes')
p.add_argument('--fruit-contact-limit-mm',type=float,default=None,help='Optional exact current-truss fruit/tool threshold; defaults to the common contact limit')
p.add_argument('--fruit-fruit-limit-mm',type=float,default=None,help='Optional exact current-truss fruit/fruit threshold; defaults to the common contact limit')
p.add_argument('--armature-setup',choices=['refresh','authored'],default='refresh',help='Authored validates the pre-warmup armature without a redundant runtime setter')
p.add_argument('--gravity-neutral-warmup',action='store_true',help='Initialize at authored rest under zero gravity, restore source gravity after fixed preload and before trial')
p.add_argument('--plant-dynamics',choices=['full','target-truss'],default='full',help='Target-truss freezes far-truss bending; different dynamics, same collision coverage')
p.add_argument('--zero-conveyor-diagnostic',action='store_true',help='Idle/pulse only: remove zero surface-velocity APIs to isolate native GPU fallback; not harvest evidence')
p.add_argument('--native-profile',action='store_true',help='Record native PhysX zones; diagnostic instrumentation may add overhead')
p.add_argument('--compiled-body-audit',action='store_true',help='CPU diagnostic: compare compiled mass/inertia, joint state and material mapping before trial')
p.add_argument('--publish-live',action='store_true',help='Publish latest measured env0 frame for a separate viewer; saved full physics history stays unchanged')
p.add_argument('--live-fps',type=float,default=10.,help='Latest-frame publication cap in wall time')
p.add_argument('--start-gate',type=Path,help='CPU pool diagnostic: publish ready.json and await the controller barrier')
p.add_argument('--gate-timeout',type=float,default=180.)
a=p.parse_args()
if not math.isfinite(a.contact_limit_mm) or not .5<=a.contact_limit_mm<=1.:
    p.error('Common contact limit must be finite within 0.5..1mm')
if a.fruit_contact_limit_mm is None:a.fruit_contact_limit_mm=a.contact_limit_mm
if a.fruit_fruit_limit_mm is None:a.fruit_fruit_limit_mm=a.contact_limit_mm
if a.native_contact_vector_copy and (not a.buffer_contact_events or a.contact_backend!='callback'):
    p.error('Native contact vector copying requires buffered callback contacts')
if not math.isfinite(a.condition_foliage_seconds) or not 0<=a.condition_foliage_seconds<=3:p.error('Conditioning budget must be finite within 0..3 seconds')
if a.condition_foliage_seconds and (a.mode!='idle' or a.num_envs!=1 or a.replicate or a.rail_to_y is not None or a.contact_backend!='callback' or not a.buffer_contact_events or not a.verify_leaf_contact_after_step or not a.save_route_checkpoint or not math.isfinite(a.seconds) or a.seconds<a.condition_foliage_seconds+1):
    p.error('Foliage conditioning requires one stationary buffered callback idle world, solved leaf verification, saved checkpoint and at least 1 second strict suffix')
if a.condition_foliage_seconds and (a.contact_limit_mm!=.5 or a.fruit_contact_limit_mm!=.5 or a.fruit_fruit_limit_mm!=.5 or a.isolate_background or a.plant_dynamics!='full' or a.zero_conveyor_diagnostic):p.error('Conditioning preserves strict 0.5mm contact criteria, complete background contacts and full source plant dynamics')
if a.allow_resume_conditioned_start and (a.resume_native_checkpoint is None or a.allow_resume_leaf_proxy_revision or a.allow_resume_accuracy_proxy_revision or a.allow_resume_attachment_transfer):p.error('Conditioned-start resume requires an exact same-model checkpoint, without a model revision')
if a.pulse_body and (a.mode!='pulse' or a.num_envs!=1 or a.replicate):p.error('--pulse-body requires one unreplicated pulse fixture')
if (a.verify_leaf_contact_after_step or a.verify_harvest_contact_after_step) and (a.num_envs!=1 or a.replicate or a.contact_backend!='callback'):
    p.error('Leaf final-geometry verification requires one unreplicated callback world')
if a.verify_structural_scene_query and not a.verify_harvest_contact_after_step:
    p.error('Structural scene queries require the exact harvest post-step geometry inventory')
if a.approach_yaw_mode=='truss-axis' and (a.mode!='harvest' or a.preparation!='horizontal_ready' or a.approach_pitch_deg!=0 or a.approach_yaw_offset_deg!=0):
    p.error('truss-axis requires harvest/horizontal_ready with zero pitch and zero yaw offset')
if a.whole_truss and (a.harvest_profile!='belt_open_close_cycle' or a.num_envs!=1 or a.replicate or a.entry_only or a.mode!='harvest'):
    p.error('--whole-truss requires one unreplicated belt_open_close_cycle harvest')
if not math.isfinite(a.last_fruit_grasp_sway_mm) or not 0<=a.last_fruit_grasp_sway_mm<=1.5:
    p.error('Last-fruit arm sway must be finite within0..1.5mm')
if a.last_fruit_grasp_sway_mm and (not a.whole_truss or a.preparation!='horizontal_ready' or a.approach_pitch_deg!=0):
    p.error('Last-fruit arm sway requires the existing horizontal whole-truss belt cycle')
if a.rail_to_y is not None and (not math.isfinite(a.rail_to_y) or a.mode not in ('harvest','idle') or a.num_envs!=1 or a.replicate):
    p.error('Physical rail travel requires one unreplicated harvest or idle trial')
if not 1<=a.max_truss_cycles<=20:p.error('Whole-truss cycle budget must be 1..20')
if not 1<=a.max_fruit_attempts<=8:p.error('Fruit attempt budget must be 1..8')
if not 0<=a.single_fruit_when_remaining<=3:p.error('Individual sparse-fruit threshold must be 0..3')
if a.harvest_group_policy is not None and (not a.whole_truss or a.truss_order!='highest-first' or a.approach_yaw_mode!='truss-axis' or a.single_fruit_when_remaining or a.split_unproductive_pairs or a.near_trusses_metadata is not None or a.last_fruit_height_offset_mm is not None or a.last_fruit_depth_mm is not None or a.last_fruit_grasp_sway_mm or a.grasp_lateral_offset_mm):
    p.error('Ripe-group pilot requires one exact truss, whole-truss, highest-first, truss-axis; disable individual cleanup, lateral offsets and near-truss continuation')
if not 2<=a.truss_axis_min_fruits<=10:p.error('Truss-axis fitting minimum must be 2..10 fruit')
if (not math.isfinite(a.truss_entry_yaw_offset_deg) or abs(a.truss_entry_yaw_offset_deg)>90 or
        (a.truss_entry_yaw_offset_deg and a.approach_yaw_mode!='truss-axis')):
    p.error('--truss-entry-yaw-offset-deg requires truss-axis and finite +/-90 degrees')
if not math.isfinite(a.sparse_entry_duration_scale) or not 1<=a.sparse_entry_duration_scale<=8:p.error('Sparse entry duration scale must be finite within 1..8')
if a.verify_background_leaf_enclosures and not a.verify_leaf_contact_after_step:p.error('Background leaf enclosure check requires solved leaf verification')
if a.capture_leaf_colliders and not a.verify_leaf_contact_after_step:p.error('Native leaf cooking capture requires solved leaf geometry reader')
if a.capture_tool_colliders and (not a.verify_leaf_contact_after_step or a.num_envs!=1 or a.replicate):p.error('Native tool cooking capture requires one unreplicated existing geometry reader')
if a.native_tool_collision_cache and (a.num_envs!=1 or a.replicate):p.error('Native tool collision cache requires one unreplicated source model')
if a.preparation_height_first and (a.mode!='harvest' or a.preparation!='horizontal_ready'):
    p.error('Height-first transfer requires harvest/horizontal_ready')
if a.preparation_gap_mm is not None and (not math.isfinite(a.preparation_gap_mm) or not 28<=a.preparation_gap_mm<=96 or a.preparation!='horizontal_ready' or a.harvest_profile not in ('belt_spin_release','belt_open_close_cycle')):
    p.error('Preparation gap requires horizontal-ready belt harvest and 28..96mm')
if not math.isfinite(a.ready_lift_offset_mm) or abs(a.ready_lift_offset_mm)>300 or (a.ready_lift_offset_mm and a.preparation!='horizontal_ready'):
    p.error('Ready lift redistribution must be within +/-300mm and requires horizontal_ready')
if not math.isfinite(a.max_aim_translation_m) or not 0<a.max_aim_translation_m<=2:p.error('Aim translation bound must be finite within (0,2]m')
if a.near_trusses_metadata is not None and not a.whole_truss:p.error('Near-truss selection requires whole-truss native sequence')
if a.initial_truss_first and a.near_trusses_metadata is None:p.error('Initial-truss priority requires near-truss inventory')
if a.resume_native_checkpoint is not None and (not a.whole_truss or a.num_envs!=1 or a.replicate or (a.rail_to_y is not None and not a.allow_resume_rail_travel)):p.error('Native cycle resume requires one whole-truss world; additional physical rail travel needs explicit --allow-resume-rail-travel')
if a.allow_resume_rail_travel and (a.resume_native_checkpoint is None or a.rail_to_y is None):p.error('Explicit resumed rail travel requires both a native checkpoint and --rail-to-y')
if a.allow_resume_leaf_proxy_revision and a.resume_native_checkpoint is None:p.error('Leaf revision resume approval requires a native checkpoint')
if a.allow_resume_accuracy_proxy_revision and a.resume_native_checkpoint is None:p.error('Accuracy revision resume approval requires a native checkpoint')
if a.allow_resume_attachment_transfer and a.resume_native_checkpoint is None:p.error('Attachment transfer requires a native checkpoint')
if sum(bool(v) for v in (a.allow_resume_leaf_proxy_revision,a.allow_resume_accuracy_proxy_revision,a.allow_resume_attachment_transfer))>1:p.error('Select one independently verified revision mode')
if a.horizontal_ready_connector!='cartesian' and (a.mode!='harvest' or a.preparation!='horizontal_ready'):
    p.error('Joint staging connector requires harvest with horizontal_ready preparation')
if a.horizontal_ready_seed_index is not None and a.horizontal_ready_connector!='joint':
    p.error('Explicit staging IK seed requires the joint connector')
if a.horizontal_ready_normalize_turns and (a.horizontal_ready_connector!='joint' or a.preparation!='horizontal_ready' or a.mode!='harvest'):
    p.error('Equivalent staging turns require the horizontal-ready joint connector in harvest mode')
if a.diagnostic_motion_plan is not None and (a.mode!='harvest' or a.preparation!='horizontal_ready' or a.horizontal_ready_connector!='joint' or not a.whole_truss or a.max_truss_cycles!=1 or a.resume_native_checkpoint is not None or a.near_trusses_metadata is not None or a.rail_to_y is not None or a.hz!=120):
    p.error('Audited diagnostic plan import requires one fresh120Hz horizontal joint whole-truss cycle, without rail/resume/scope changes')
if a.split_unproductive_pairs and not a.whole_truss:p.error('Pair centering recovery requires whole-truss mode')
if not math.isfinite(a.grasp_depth_mm) or not 15<=a.grasp_depth_mm<=100 or (a.grasp_depth_mm!=65 and a.harvest_profile not in ('belt_spin_release','belt_open_close_cycle')):
    p.error('Grasp depth must be 15..100 mm and overrides require a belt profile')
if not math.isfinite(a.entry_retract_mm) or not 50<=a.entry_retract_mm<=300 or (a.entry_retract_mm!=100 and (a.preparation!='horizontal_ready' or a.harvest_profile not in ('belt_spin_release','belt_open_close_cycle'))):
    p.error('Entry retraction must be finite50..300mm; overrides require horizontal belt preparation')
if a.later_grasp_depth_mm is not None and (not math.isfinite(a.later_grasp_depth_mm) or not 15<=a.later_grasp_depth_mm<=100 or not a.whole_truss):
    p.error('Later grasp depth requires whole-truss belt-cycle mode and finite15..100mm')
if a.belt_off_control and a.harvest_profile not in ('belt_spin_release','belt_open_close_cycle'):p.error('--belt-off-control requires a belt harvest profile')
if not math.isfinite(a.grasp_gap_mm) or not 28<=a.grasp_gap_mm<=96 or (a.grasp_gap_mm!=28 and a.harvest_profile not in ('belt_spin_release','belt_open_close_cycle')):
    p.error('Grasp gap must be 28..96 mm and overrides require a belt harvest profile')
if not math.isfinite(a.grasp_height_offset_mm) or abs(a.grasp_height_offset_mm)>50 or (a.grasp_height_offset_mm and a.harvest_profile not in ('belt_spin_release','belt_open_close_cycle')):
    p.error('Grasp height offset must be within +/-50 mm and requires a belt harvest profile')
if ((a.last_fruit_height_offset_mm is not None or a.last_fruit_depth_mm is not None) and
        (not a.whole_truss or a.harvest_profile!='belt_open_close_cycle')):
    p.error('Last-fruit aim requires the existing whole-truss belt cycle')
if a.last_fruit_height_offset_mm is not None and (not math.isfinite(a.last_fruit_height_offset_mm) or abs(a.last_fruit_height_offset_mm)>50):
    p.error('Last-fruit height must be finite within +/-50mm')
if a.last_fruit_depth_mm is not None and (not math.isfinite(a.last_fruit_depth_mm) or not 15<=a.last_fruit_depth_mm<=100):
    p.error('Last-fruit depth must be finite within15..100mm')
if not math.isfinite(a.grasp_lateral_offset_mm) or abs(a.grasp_lateral_offset_mm)>20 or (a.grasp_lateral_offset_mm and (a.preparation!='horizontal_ready' or a.harvest_profile not in ('belt_spin_release','belt_open_close_cycle'))):
    p.error('Grasp lateral offset must be within +/-20 mm and requires horizontal-ready belt entry')
if any(not math.isfinite(v) or v<.5 for v in (a.fruit_contact_limit_mm,a.fruit_fruit_limit_mm)):p.error('Fruit contact tolerance must be finite and at least 0.5mm')
if (a.fruit_contact_limit_mm!=a.contact_limit_mm or a.fruit_fruit_limit_mm!=a.contact_limit_mm) and (a.contact_backend!='callback' or a.num_envs!=1 or a.replicate):
    p.error('Fruit contact override requires exact native callback mapping in one unreplicated environment')
if a.scene_ccd and a.device!='cpu':p.error('--scene-ccd is validated here only for CPU trials')
if (a.verify_rail_environment_preflight or a.verify_preparation_environment_preflight or a.verify_full_environment_preflight) and (a.hz not in (60,120) or a.num_envs!=1 or a.replicate):
    p.error('Rail environment preflight requires one unreplicated60/120Hz environment')
if a.entry_only and (a.mode!='harvest' or a.preparation!='horizontal_ready'):
    p.error('--entry-only requires --mode harvest --preparation horizontal_ready')
if a.harvest_profile in ('belt_spin_release','belt_open_close_cycle') and (a.mode!='harvest' or a.preparation!='horizontal_ready' or a.entry_only):
    p.error('--harvest-profile belt_spin_release requires harvest/horizontal_ready without --entry-only')
if not math.isfinite(a.belt_speed_m_s) or a.belt_speed_m_s<0 or not math.isfinite(a.belt_seconds) or a.belt_seconds<=0:
    p.error('Differential belt speed must be finite and nonnegative; duration must be finite and positive')
if not math.isfinite(a.close_seconds) or a.close_seconds<=0 or (a.close_seconds!=3. and a.harvest_profile not in ('belt_spin_release','belt_open_close_cycle')):
    p.error('Positive close duration override requires a belt harvest profile')
if not math.isfinite(a.belt_ramp_seconds) or not 0<=a.belt_ramp_seconds<=a.belt_seconds/2 or (a.belt_ramp_seconds and a.harvest_profile not in ('belt_spin_release','belt_open_close_cycle')):
    p.error('Belt ramp requires a belt harvest profile and must be between zero and half the spin duration')
if not math.isfinite(a.belt_infeed_speed_m_s) or a.belt_infeed_speed_m_s<0 or (a.belt_infeed_speed_m_s and a.harvest_profile not in ('belt_spin_release','belt_open_close_cycle')):
    p.error('Nonnegative belt infeed override requires a belt harvest profile')
if a.belt_speed_m_s==0 and (a.harvest_profile not in ('belt_spin_release','belt_open_close_cycle') or a.belt_infeed_speed_m_s==0):
    p.error('Zero differential speed requires a belt harvest profile with positive inward speed')
if not math.isfinite(a.approach_pitch_deg) or not -90<=a.approach_pitch_deg<=90:
    p.error('--approach-pitch-deg must be finite and within -90..90')
if not math.isfinite(a.approach_yaw_offset_deg) or not -180<=a.approach_yaw_offset_deg<=180:
    p.error('--approach-yaw-offset-deg must be finite and within -180..180')
if not 1<=a.gpu_pair_capacity<=2147483647:
    p.error('--gpu-pair-capacity must be 1..2147483647')
if a.replication_persist_layer_stack and (not a.replicate or not 1<=a.num_envs<=8):
    p.error('--replication-persist-layer-stack requires --replicate and 1..8 environments')
if a.replication_defer_gpu_bootstrap and (not a.replication_persist_layer_stack or a.device!='cuda:0'):
    p.error('--replication-defer-gpu-bootstrap requires CUDA and --replication-persist-layer-stack')
a.output.mkdir(parents=True,exist_ok=False)
code_dir=a.output/'code';code_dir.mkdir()
code_hashes={}
for source in Path(__file__).parent.glob('*.py'):
    shutil.copy2(source,code_dir/source.name)
    code_hashes[source.name]=hashlib.sha256(source.read_bytes()).hexdigest()
inventory_source=Path(__file__).parent/'validation/passive_s300_chassis_source_inventory.json'
if inventory_source.is_file():
    (code_dir/'validation').mkdir()
    shutil.copy2(inventory_source,code_dir/'validation'/inventory_source.name)
    code_hashes['validation/'+inventory_source.name]=hashlib.sha256(inventory_source.read_bytes()).hexdigest()
(a.output/'code_sha256.json').write_text(json.dumps(code_hashes,indent=2))
start_wall=time.perf_counter();exit_code=0
live=None;warmup=None
from motion_profile import MotionProfile
profile=MotionProfile(a.output/'profile.json')
from isaacsim import SimulationApp
from motion_app import create_motion_app
app=create_motion_app(SimulationApp,headless=a.headless,width=1280,height=800,
    performance=a.performance,physics_only=a.physics_only)
try:
    import numpy as np
    import torch
    import carb
    import omni.usd
    if not a.headless:import omni.kit.viewport.utility as vu
    extension_manager=app._app.get_extension_manager()
    (a.output/'app_extensions.json').write_text(json.dumps(dict(physics_only=a.physics_only,
        enabled=[x for x in extension_manager.get_extensions() if x.get('enabled')]),indent=2))
    from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,PhysicsSchemaTools,Gf
    from scipy.spatial.transform import Rotation as R
    from isaacsim.core.simulation_manager import SimulationManager as sim
    from omni.physx import get_physx_simulation_interface,get_physx_interface
    from omni.physx.bindings._physx import SimulationEvent
    from motion_kinematics import cycle_opening_check, ToolKinematics,plan_motion,belt_speed_fraction
    from motion_assets import ROBOT,sha

    manifest=json.loads((a.assembly/'motion_manifest.json').read_text())
    if manifest.get('elastic_neighbors') and (a.num_envs!=1 or a.replicate or a.resume_native_checkpoint or a.plant_dynamics!='full' or a.isolate_background):
        raise ValueError('Neighbor comparison requires one full world, no replication/resume/isolation')
    scene=a.assembly/'scene.usda'
    if sha(scene)!=manifest['scene_sha256']:raise ValueError('Scene hash mismatch')
    from compact_motion_assets import verify_compact_dependencies
    compact_dependencies=verify_compact_dependencies(a.assembly,manifest)
    from native_stage_selection import select_native_scene
    native_stage_selection=select_native_scene(a.assembly,use_materialized=a.native_materialized_stage)
    native_scene=Path(native_stage_selection['native_scene'])
    (a.output/'native_stage_selection.json').write_text(json.dumps(native_stage_selection,indent=2))
    if not omni.usd.get_context().open_stage(str(native_scene.resolve())):raise RuntimeError('Stage open failed')
    stage=omni.usd.get_context().get_stage()
    resume_context=None
    if a.resume_native_checkpoint is not None:
        from resume_native_state import prepare_resume,imported_break_markers
        resume_context=prepare_resume(stage,a.assembly,manifest,a.resume_native_checkpoint,
            allow_leaf_proxy_revision=a.allow_resume_leaf_proxy_revision,
            allow_accuracy_proxy_revision=a.allow_resume_accuracy_proxy_revision,
            allow_attachment_transfer=a.allow_resume_attachment_transfer,
            allow_conditioned_start=a.allow_resume_conditioned_start,
            output_json=a.output/'resume_input.json')
    route_start_context=None
    from motion_continuation import route_restore_selection,measured_resume_rail_origin,attach_resume_rail_hold,resumed_rail_destination,attach_resumed_rail_travel
    route_restore=route_restore_selection(manifest,resume_context,num_envs=a.num_envs,replicate=a.replicate)
    if route_restore['superseded_archive'] is not None:
        (a.output/'route_start_superseded.json').write_text(json.dumps(route_restore['superseded_archive'],indent=2))
    if route_restore['prepare_route_start']:
        from native_route_start import prepare_route_start
        route_start_context=prepare_route_start(stage,a.assembly,manifest)
        if a.mode=='idle' and route_start_context['controller_state']['position_targets'] is None:
            raise ValueError('Route idle requires proven original controller position targets; measured q is not a hold setpoint')
        (a.output/'route_start_input.json').write_text(json.dumps(route_start_context['report'],indent=2))
    from motion_stage_metrics import stage_metrics,require_motion_units
    actual_stage_metrics=stage_metrics(stage)
    (a.output/'stage_metrics.json').write_text(json.dumps(actual_stage_metrics,indent=2))
    require_motion_units(stage)
    robot_self_collision_enabled=stage.GetPrimAtPath(ROBOT+'/root_joint').GetAttribute('physxArticulation:enabledSelfCollisions').Get() is True
    if a.mode=='harvest' or a.rail_to_y is not None:
        from motion_self_collision import require_robot_self_collisions
        require_robot_self_collisions(stage)
    from motion_app import configure_scene_device
    bootstrap_device='cpu' if a.replication_defer_gpu_bootstrap else a.device
    bootstrap_broadphase='MBP' if a.replication_defer_gpu_bootstrap else a.broadphase
    if a.replication_defer_gpu_bootstrap:
        # The source itself authors GPU dynamics. Switch before any app.update,
        # and do not carry old-stage Fabric state into the reopened clone.
        sim.set_default_physics_scene('/World/PhysicsScene')
        sim.set_device('cpu');sim.enable_fabric(False)
        configure_scene_device(stage,'cpu',broadphase='MBP')
    if a.performance=='optimized':
        from motion_visuals import prepare_display_meshes
        (a.output/'display_schema_cleanup.json').write_text(json.dumps(prepare_display_meshes(stage),indent=2))
    belt_fixture=None
    if a.mode=='belt_control':
        from motion_belt_control import build_belt_control
        belt_fixture=build_belt_control(stage,manifest)
        (a.output/'belt_fixture.json').write_text(json.dumps(belt_fixture,indent=2))
    resolution=None
    if a.plant_dynamics=='target-truss':
        from motion_resolution import lock_far_trusses
        resolution=lock_far_trusses(stage,manifest)
        (a.output/'resolution.json').write_text(json.dumps(resolution,indent=2))
    if a.zero_conveyor_diagnostic:
        from motion_gpu_diagnostics import disable_zero_surface_velocity_for_diagnostic
        conveyor_diagnostic=disable_zero_surface_velocity_for_diagnostic(stage,mode=a.mode)
        (a.output/'zero_conveyor_diagnostic.json').write_text(json.dumps(conveyor_diagnostic,indent=2))
    isolated_count=0
    if a.collision_filter=='groups':
        from motion_collision_groups import compress_leaf_collision_policy
        compression=compress_leaf_collision_policy(stage,isolate_background=a.isolate_background)
        (a.output/'collision_filter_compression.json').write_text(json.dumps(compression,indent=2))
        if a.isolate_background:
            disabled=[str(prim.GetPath()) for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/Greenhouse/Plants'),Usd.TraverseInstanceProxies())
                      if prim.HasAPI(UsdPhysics.CollisionAPI)]
            isolated_count=len(disabled)
            (a.output/'fixture_disabled_background_colliders.json').write_text(json.dumps(disabled))
    elif a.isolate_background:
        # Opt-in negative control, not the default farm trial. Preserve all
        # geometry and log the exact disabled paths. Originals are never saved.
        plants=stage.GetPrimAtPath('/World/Greenhouse/Plants')
        disabled=[str(prim.GetPath()) for prim in Usd.PrimRange(plants,Usd.TraverseInstanceProxies())
                  if prim.HasAPI(UsdPhysics.CollisionAPI)]
        bg=UsdPhysics.CollisionGroup.Define(stage,'/World/FixtureGroups/Background')
        bg.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(['/World/Greenhouse/Plants'])
        active=UsdPhysics.CollisionGroup.Define(stage,'/World/FixtureGroups/Active')
        active.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(['/World/ElasticTarget','/World/ActuatedTool','/World/HarvestRobot'])
        bg.CreateFilteredGroupsRel().SetTargets([active.GetPath()])
        # The supplied dense decoration also intersects its own truss branches.
        # Diagnostic control excludes only appendage vs plant interactions;
        # arm/tool vs appendages, rods, fruits and house remain physical.
        appendages=[];other_plant=[]
        for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/ElasticTarget')):
            if not prim.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:continue
            (appendages if '/Appendage_' in str(prim.GetPath()) else other_plant).append(prim)
        for leaf in appendages:
            rel=UsdPhysics.FilteredPairsAPI.Apply(leaf).CreateFilteredPairsRel()
            for other in other_plant:rel.AddTarget(other.GetPath())
        (a.output/'fixture_internal_leaf_filters.json').write_text(json.dumps(dict(
            leaves=[str(x.GetPath()) for x in appendages],other_colliders=[str(x.GetPath()) for x in other_plant],
            reason='component diagnostic only; authored own-leaf/branch overlap; not farm validity')))
        isolated_count=len(disabled)
        (a.output/'fixture_disabled_background_colliders.json').write_text(json.dumps(disabled))
    for _ in range(5):app.update()
    kin=ToolKinematics(stage)
    rail_destination=None;planning_kin=kin
    resume_rail_origin=measured_resume_rail_origin(manifest,resume_context)
    if resume_rail_origin is not None:
        from rail_motion import station_kinematics,rail_command
        planning_kin=station_kinematics(kin,resume_rail_origin)
        (a.output/'resume_rail_origin.json').write_text(json.dumps(resume_rail_origin,indent=2))
    if a.rail_to_y is not None:
        from rail_motion import rail_destination as get_rail_destination,station_kinematics,attach_rail_commands,rail_command
        if resume_context is not None:
            if resume_rail_origin is None:raise ValueError('Resumed rail travel requires the measured source physical rail')
            rail_destination=resumed_rail_destination(manifest,a.rail_to_y,a.rail_speed_m_s,resume_rail_origin)
        else:rail_destination=get_rail_destination(manifest,a.rail_to_y,a.rail_speed_m_s)
        planning_kin=station_kinematics(kin,rail_destination)
    recorded_prefix=None
    if a.preparation_prefix_plan is not None:
        if a.mode!='harvest':raise ValueError('--preparation-prefix-plan requires harvest mode')
        prefix_summary=json.loads((a.preparation_prefix_plan.parent/'summary.json').read_text())
        if prefix_summary['model_sha256']!=sha(scene):raise ValueError('Recorded preparation prefix model hash mismatch')
        prefix_plan=json.loads(a.preparation_prefix_plan.read_text())
        recorded_prefix=[(p['name'],p['duration_s'],np.asarray(p['waypoint_q']),p['end_gap_m'],*p['belt_local_y_m_s'])
                         for p in prefix_plan['phases'][:2]]
        prefix_input=dict(plan=str(a.preparation_prefix_plan.resolve()),plan_sha256=sha(a.preparation_prefix_plan),
                          model_sha256=prefix_summary['model_sha256'],reused_phases=[p[0] for p in recorded_prefix],
                          scope='Recorded nominal prefix commands; actual physics rerun at normal start')
        (a.output/'preparation_prefix_input.json').write_text(json.dumps(prefix_input,indent=2))
    initial_robot_q=np.asarray(manifest.get('initial_robot_q',np.zeros(7)),dtype=float)
    initial_inventory_breaks=[];initial_body_paths=None;initial_body_poses=None;initial_attempt_counts={}
    if resume_context is not None:
        state=resume_context['arrays'];saved_names=list(map(str,state['robot_dof_names']))
        initial_robot_q=np.array([state['robot_q'][saved_names.index(name)] for name in kin.names])
        initial_inventory_breaks=imported_break_markers(resume_context['metadata'])
        initial_body_paths=list(map(str,state['body_paths']));initial_body_poses=state['body_poses_world']
        previous_sequence=resume_context['metadata']['sequence']
        initial_attempt_counts=dict(previous_sequence.get('attempt_counts',{}))
        if not initial_attempt_counts:
            for previous_cycle in previous_sequence['cycle_plans']:
                for fruit_name in previous_cycle['fruits']:
                    initial_attempt_counts[fruit_name]=initial_attempt_counts.get(fruit_name,0)+1
    from truss_sequence import truss_fruits,select_group
    sequence_fruits=truss_fruits(manifest);sequence_selector=select_group;near_truss_scope=None
    if (manifest.get('plant_interaction_variant',{}).get('mode')=='target_truss_and_own_stem_only'
            and a.near_trusses_metadata is not None):
        raise ValueError('Target-only collision assets require one exact target truss per active window; broader near-truss selection would include disabled colliders')
    if a.near_trusses_metadata is not None:
        from row_harvest_targets import scoped_trusses,eligible_truss_fruits,select_scoped_group,group_truss_fruits
        near_truss_scope=scoped_trusses(manifest,json.loads(a.near_trusses_metadata.read_text()))
        sequence_fruits=eligible_truss_fruits(manifest,near_truss_scope)
        if not sequence_fruits:raise ValueError('No near-side trusses in this active plant')
        sequence_selector=select_scoped_group
        (a.output/'near_truss_scope.json').write_text(json.dumps(near_truss_scope,indent=2))
    reference_center=np.array(manifest.get('grasp_reference',manifest['target'])['center'])
    initial_axis=None
    if a.approach_yaw_mode=='truss-axis':
        from truss_sequence import truss_fruits,horizontal_truss_direction,entry_direction
        axis_fruits=truss_fruits(manifest)
        # Entry happens at the planned rail station. Using the departure base
        # can reverse the PCA axis when the next truss is farther along the row.
        axis_hint=reference_center-planning_kin.world[:3,3]
        if resume_context is not None and resume_context.get('checkpoint_kind')!='canonical_conditioned_start':
            from resume_axis import resolve_resume_axis
            previous_axis,axis_provenance=resolve_resume_axis(resume_context,a.resume_native_checkpoint)
            axis_hint=np.asarray(previous_axis['direction_world'],dtype=float)
            (a.output/'resume_axis_provenance.json').write_text(json.dumps(axis_provenance,indent=2))
        initial_axis=horizontal_truss_direction(axis_fruits,initial_inventory_breaks,
            initial_body_paths or [f['path'] for f in axis_fruits],
            initial_body_poses if initial_body_poses is not None else np.array([f['center'] for f in axis_fruits]),
            axis_hint,fallback_direction=axis_hint,min_attached_fruits=a.truss_axis_min_fruits)
        print('[TRUSS YAW] preliminary manifest-truss direction before actual group selection',initial_axis['direction_world'],
              'world yaw deg',initial_axis['yaw_world_deg'],flush=True)
    last_axis=initial_axis
    group_policy_selector=None
    if a.harvest_group_policy is not None:
        from harvest_group_policy import load_policy,HarvestGroupSelector
        group_policy=load_policy(a.harvest_group_policy,a.assembly,sequence_fruits)
        shutil.copyfile(a.harvest_group_policy,a.output/'harvest_group_policy.json')
        def group_direction(key,fruits,breaks,paths,poses):
            natural_axis=horizontal_truss_direction(fruits,breaks,paths,poses,
                last_axis['direction_world'],fallback_direction=last_axis['direction_world'],
                min_attached_fruits=a.truss_axis_min_fruits)
            return entry_direction(natural_axis,a.truss_entry_yaw_offset_deg)
        group_policy_selector=HarvestGroupSelector(group_policy,group_direction,
            depth_m=a.grasp_depth_mm/1000,
            height_offset_m=manifest['grasp_reference'].get('height_offset_m',0.)+a.grasp_height_offset_mm/1000,
            output=a.output/'group_selection_history.json')
        sequence_selector=group_policy_selector
    prior_completed_native_cycles=0
    initial_grasp_depth_mm=a.grasp_depth_mm;initial_depth_evidence=None
    if a.later_grasp_depth_mm is not None:
        from grasp_depth_schedule import completed_native_cycles,grasp_depth_for_cycle
        if resume_context is not None:
            prior_completed_native_cycles=completed_native_cycles(resume_context['metadata']['sequence']['cycle_plans'])
        initial_grasp_depth_mm,initial_depth_evidence=grasp_depth_for_cycle(
            a.grasp_depth_mm,a.later_grasp_depth_mm,prior_completed_native_cycles)
    if group_policy_selector is not None:group_policy_selector.depth_m=initial_grasp_depth_mm/1000
    def make_plan(translation=(0.,0.,0.),axis=None,move_rail_first=False,grasp_depth_mm=None,depth_evidence=None,grasp_height_mm=None,last_aim_evidence=None):
        from truss_sequence import entry_direction
        effective_depth_mm=initial_grasp_depth_mm if grasp_depth_mm is None else grasp_depth_mm
        effective_height_mm=a.grasp_height_offset_mm if grasp_height_mm is None else grasp_height_mm
        result,metadata=plan_motion(planning_kin,reference_center,start=initial_robot_q,preparation=a.preparation,
            pitch_deg=a.approach_pitch_deg,yaw_offset_deg=a.approach_yaw_offset_deg,
            recorded_prefix=recorded_prefix,entry_only=a.entry_only,harvest_profile=a.harvest_profile,
            belt_speed_m_s=a.belt_speed_m_s,belt_seconds=a.belt_seconds,close_seconds=a.close_seconds,
            belt_ramp_seconds=a.belt_ramp_seconds,belt_infeed_speed_m_s=a.belt_infeed_speed_m_s,
            grasp_gap_m=a.grasp_gap_mm/1000,grasp_height_offset_m=effective_height_mm/1000,
            grasp_lateral_offset_m=a.grasp_lateral_offset_mm/1000,
            grasp_translation_world_m=translation,preparation_height_first=a.preparation_height_first,
            grasp_depth_m=effective_depth_mm/1000,
            approach_distance_m=a.entry_retract_mm/1000,
            preparation_gap_m=None if a.preparation_gap_mm is None else a.preparation_gap_mm/1000,
            preparation_opening=a.preparation_opening,
            ready_lift_offset_m=a.ready_lift_offset_mm/1000,
            max_grasp_translation_m=a.max_aim_translation_m,
            allow_measured_ready_start=a.measured_ready_start,
            horizontal_ready_connector=a.horizontal_ready_connector,
            horizontal_ready_seed_index=a.horizontal_ready_seed_index,
            horizontal_ready_normalize_turns=a.horizontal_ready_normalize_turns,
            approach_direction_world=entry_direction(axis,a.truss_entry_yaw_offset_deg) if axis is not None else None)
        if a.later_grasp_depth_mm is not None:
            metadata['grasp_depth_schedule']=depth_evidence if depth_evidence is not None else initial_depth_evidence
        if last_aim_evidence is not None:metadata['last_remaining_fruit_aim']=last_aim_evidence
        if axis is not None:metadata['truss_axis']=axis
        if axis is not None and a.truss_entry_yaw_offset_deg:
            metadata['truss_entry_yaw_offset_deg']=a.truss_entry_yaw_offset_deg
            metadata['entry_orientation_policy']='explicit_world_z_offset_from_truss_axis'
        if a.sparse_entry_duration_scale!=1. and axis is not None and axis.get('fallback_reason') in ('insufficient_attached_fruits_for_longitudinal_axis','fewer_than_two_attached_fruits'):
            from motion_timing import retime_entry
            result,metadata=retime_entry(result,metadata,a.sparse_entry_duration_scale)
        if rail_destination is not None:
            if resume_rail_origin is not None:
                result,metadata=attach_resumed_rail_travel(result,metadata,initial_robot_q,rail_destination,resume_rail_origin,move_first=move_rail_first)
            else:result,metadata=attach_rail_commands(result,metadata,initial_robot_q,rail_destination,move_first=move_rail_first)
        elif resume_rail_origin is not None:
            result,metadata=attach_resume_rail_hold(result,metadata,resume_rail_origin)
        return result,metadata
    physical_unproductive_pairs=set()
    if a.split_unproductive_pairs and resume_context is not None:
        from truss_sequence import unproductive_pairs
        prior=resume_context['metadata']['sequence']
        physical_unproductive_pairs=unproductive_pairs(prior['cycle_plans'],prior['fruit_joints'])
        selected_names={f['name'] for f in sequence_fruits}
        physical_unproductive_pairs={pair for pair in physical_unproductive_pairs if set(pair)<=selected_names}
    def selection_evidence(group):
        if group is None or not a.split_unproductive_pairs:return group
        evidence=group.get('planning_rejection_alternative')
        if evidence is not None and physical_unproductive_pairs:
            evidence['scope']='Explicit completed-physics pair evidence plus same-boundary planning alternatives; no reachability impossibility implied'
            group['completed_pair_centering_recovery']=dict(
                exact_pairs=[list(pair) for pair in sorted(physical_unproductive_pairs)],
                individual_parent=evidence.get('parent_group'),native_joint_detachment_required=True,
                attempt_budget_unchanged=True)
        return group
    initial_sequence_group=None;initial_translation=(0.,0.,0.);verified_truss_axes={}
    previous_cycle_records=[] if resume_context is None else resume_context['metadata']['sequence']['cycle_plans']
    if a.near_trusses_metadata is not None and resume_context is not None:
        from row_harvest_targets import truss_key
        previous=resume_context['metadata']['sequence']
        for axis in [previous.get('initial_axis')]+[r.get('truss_axis') for r in previous_cycle_records]:
            if axis is None or not axis.get('fruit_names'):continue
            keys={truss_key(name) for name in axis['fruit_names']}
            if len(keys)!=1:raise ValueError('Saved truss axis crosses exact truss identities')
            verified_truss_axes[next(iter(keys))]=axis
    if a.whole_truss and (a.truss_order=='highest-first' or a.near_trusses_metadata is not None or resume_context is not None):
        from truss_sequence import truss_fruits,select_group
        authored_fruits=sequence_fruits
        initial_fruits=truss_fruits(manifest) if a.initial_truss_first else authored_fruits
        if not set(f['name'] for f in initial_fruits)<=set(f['name'] for f in authored_fruits):
            raise ValueError('Initial target truss is excluded by the near-side placement policy')
        if a.near_trusses_metadata is not None and a.initial_truss_first:
            from truss_sequence import select_near_scope_continuation
            from row_harvest_targets import truss_key
            initial_sequence_group,selection_provenance=select_near_scope_continuation(
                authored_fruits,initial_inventory_breaks,
                initial_body_paths or [f['path'] for f in authored_fruits],
                initial_body_poses if initial_body_poses is not None else np.array([f['center'] for f in authored_fruits]),
                initial_attempt_counts,truss_key(initial_fruits[0]['name']),
                max_attempts=a.max_fruit_attempts,order=a.truss_order,
                single_fruit_when_remaining=a.single_fruit_when_remaining,
                rejected_groups=physical_unproductive_pairs)
            (a.output/'initial_near_scope_selection.json').write_text(json.dumps(selection_provenance,indent=2))
        else:
            initial_sequence_group=sequence_selector(initial_fruits,initial_inventory_breaks,
                initial_body_paths or [f['path'] for f in initial_fruits],
                initial_body_poses if initial_body_poses is not None else np.array([f['center'] for f in initial_fruits]),
                initial_attempt_counts,max_attempts=a.max_fruit_attempts,order=a.truss_order,
                single_fruit_when_remaining=a.single_fruit_when_remaining,
                rejected_groups=[pair for pair in physical_unproductive_pairs
                    if set(pair)<=set(f['name'] for f in initial_fruits)])
        selection_evidence(initial_sequence_group)
        if initial_sequence_group is None:
            if getattr(a,'harvest_group_policy',None) is not None:
                (a.output/'summary.json').write_text(json.dumps(dict(
                    completed=True,execution_scope='selection_only_no_physics',simulated_s=0.,
                    selection=group_policy_selector.last_decision,training_eligible=False,harvest_success=None),indent=2))
                print('[TRUSS] no qualifying group; skipped without a physical attempt',flush=True)
                raise SystemExit(0)
            raise ValueError('Native resume/initial selection has no remaining group')
        authored_center=np.array(initial_sequence_group.get('aim_world_m',initial_sequence_group['measured_centroid_world_m']))
        authored_center[2]+=manifest['grasp_reference'].get('height_offset_m',0.)
        initial_translation=authored_center-reference_center
        if a.near_trusses_metadata is not None and initial_axis is not None:
            from truss_sequence import near_scope_truss_direction
            initial_axis=near_scope_truss_direction(initial_sequence_group,sequence_fruits,
                initial_inventory_breaks,initial_body_paths or [f['path'] for f in sequence_fruits],
                initial_body_poses if initial_body_poses is not None else np.array([f['center'] for f in sequence_fruits]),
                planning_kin.world[:3,3],verified_truss_axes,min_attached_fruits=a.truss_axis_min_fruits)
            verified_truss_axes[initial_axis['truss_key']]=initial_axis
            last_axis=initial_axis
        if a.near_trusses_metadata is not None and a.later_grasp_depth_mm is not None:
            from grasp_depth_schedule import grasp_depth_for_truss_cycle
            initial_grasp_depth_mm,initial_depth_evidence=grasp_depth_for_truss_cycle(
                a.grasp_depth_mm,a.later_grasp_depth_mm,previous_cycle_records,
                initial_sequence_group['truss_key'])
    initial_height_mm=a.grasp_height_offset_mm;initial_last_aim=None
    if a.last_fruit_height_offset_mm is not None or a.last_fruit_depth_mm is not None:
        from single_remaining_aim import selected_aim
        initial_height_mm,initial_grasp_depth_mm,initial_last_aim=selected_aim(
            sequence_fruits,initial_inventory_breaks,initial_sequence_group,
            height_mm=a.grasp_height_offset_mm,depth_mm=initial_grasp_depth_mm,
            last_height_mm=a.last_fruit_height_offset_mm,last_depth_mm=a.last_fruit_depth_mm)
    phases,planning=make_plan(initial_translation,initial_axis,move_rail_first=True,
        grasp_height_mm=initial_height_mm,last_aim_evidence=initial_last_aim) if a.mode=='harvest' else ([],{})
    if a.last_fruit_grasp_sway_mm:
        from last_fruit_grasp_sway import apply_last_fruit_grasp_sway
        phases,planning=apply_last_fruit_grasp_sway(planning_kin,phases,planning,
            amplitude_m=a.last_fruit_grasp_sway_mm/1000,fruits=sequence_fruits,
            breaks=initial_inventory_breaks,group=initial_sequence_group,hz=a.hz)
    if a.diagnostic_motion_plan is not None:
        from diagnostic_motion_plan import load_connected_plan
        phases,planning=load_connected_plan(a.assembly,a.diagnostic_motion_plan,phases,planning,native_tool_cache=a.native_tool_collision_cache)
        print('[MOTION] imported one audited preparation connection; manipulation suffix unchanged',flush=True)
    if a.mode=='idle' and rail_destination is not None:
        q=initial_robot_q.copy()
        phases=[('rail_hold',1.,np.array([q,q]),.044,0.,0.)]
        planning=dict(phases=[dict(name='rail_hold',duration_s=1.,waypoint_q=[q.tolist(),q.tolist()],
            end_gap_m=.044,belt_local_y_m_s=[0.,0.])])
        phases,planning=attach_rail_commands(phases,planning,q,rail_destination,move_first=True)
    if a.mode=='harvest':planning['grasp_reference']=manifest.get('grasp_reference')
    (a.output/'plan.json').write_text(json.dumps(planning,indent=2))
    if a.mode=='harvest' or a.rail_to_y is not None:
        from motion_self_collision import audit_external
        print('[MOTION] checking full arm/tool path, including jaw travel',flush=True)
        audit_external(a.assembly,a.output/'self_collision_preflight.json',plan=a.output/'plan.json',hz=a.hz,native_tool_cache=a.native_tool_collision_cache)
        if a.verify_rail_environment_preflight and any(phase['name']=='rail_move' for phase in planning.get('phases',[])):
            from rail_environment_preflight import audit_external_rail
            print('[MOTION] checking initial physical rail path against farm obstacles',flush=True)
            rail_environment_report=audit_external_rail(a.assembly,a.output/'rail_environment_preflight.json',
                plan=a.output/'plan.json',checkpoint=a.resume_native_checkpoint,hz=a.hz,native_hz=a.hz,
                allowed_truss_key=None if initial_sequence_group is None else initial_sequence_group.get('truss_key'),
                released_fruit_collection_diagnostic=a.released_fruit_ground_diagnostic,
                released_fruit_gripper_support_diagnostic=a.released_fruit_gripper_support_diagnostic,
                released_fruit_same_truss_diagnostic=a.released_fruit_same_truss_diagnostic,
                allow_attachment_transfer=a.allow_resume_attachment_transfer,
                wheel_floor_overlap_limit_m=min(.001,a.contact_limit_mm/1000),
                native_tool_cache=a.native_tool_collision_cache)
            print('[MOTION] rail environment',rail_environment_report['status'],flush=True)
        if a.verify_preparation_environment_preflight:
            from rail_environment_preflight import audit_external_rail
            print('[MOTION] checking initial preparation against the full farm',flush=True)
            audit_external_rail(a.assembly,a.output/'preparation_environment_preflight.json',
                plan=a.output/'plan.json',checkpoint=a.resume_native_checkpoint,hz=a.hz,native_hz=a.hz,
                allowed_truss_key=None if initial_sequence_group is None else initial_sequence_group.get('truss_key'),
                released_fruit_collection_diagnostic=a.released_fruit_ground_diagnostic,
                released_fruit_gripper_support_diagnostic=a.released_fruit_gripper_support_diagnostic,
                released_fruit_same_truss_diagnostic=a.released_fruit_same_truss_diagnostic,
                allow_attachment_transfer=a.allow_resume_attachment_transfer,
                wheel_floor_overlap_limit_m=min(.001,a.contact_limit_mm/1000),
                native_tool_cache=a.native_tool_collision_cache,
                scope='preparation',max_distance_queries=2000000)
        if a.verify_full_environment_preflight:
            from rail_environment_preflight import audit_external_rail
            print('[MOTION] checking all commanded phases against the full farm',flush=True)
            audit_external_rail(a.assembly,a.output/'full_environment_preflight.json',
                plan=a.output/'plan.json',checkpoint=a.resume_native_checkpoint,hz=a.hz,native_hz=a.hz,
                allowed_truss_key=None if initial_sequence_group is None else initial_sequence_group.get('truss_key'),
                released_fruit_collection_diagnostic=a.released_fruit_ground_diagnostic,
                released_fruit_gripper_support_diagnostic=a.released_fruit_gripper_support_diagnostic,
                released_fruit_same_truss_diagnostic=a.released_fruit_same_truss_diagnostic,
                allow_attachment_transfer=a.allow_resume_attachment_transfer,
                wheel_floor_overlap_limit_m=min(.001,a.contact_limit_mm/1000),
                native_tool_cache=a.native_tool_collision_cache,
                scope='full',max_distance_queries=4000000)
        if a.mode=='harvest':
            print('[MOTION] fixed entry pitch',planning['pitch_deg'],'yaw offset',planning['yaw_offset_deg'],
                  'world direction',planning['approach_direction_world'],flush=True)
            if planning.get('preparation_opening'):
                print('[MOTION] gripper opening',planning['preparation_opening'],flush=True)
    envs=[dict(index=0,root='/World',robot=ROBOT+'/root_joint',plant='/World/ElasticTarget',
               tool='/World/ActuatedTool/fixed',target=manifest['target']['path'],
               target_joint=manifest['target']['joint'],belts=manifest['tool']['belts'],mapping={})]
    print('[MOTION] model loaded; initialize native PhysX',flush=True)
    # Register the existing composed scene before deprecated global setters.
    # Otherwise a minimal app can silently create a second /PhysicsScene and
    # apply dt/device to that empty scene rather than the farm's actual scene.
    sim.set_default_physics_scene('/World/PhysicsScene')
    sim.set_device(bootstrap_device);sim.set_physics_dt(1/a.hz);sim.enable_fabric(not a.replication_defer_gpu_bootstrap)
    # The minimal worker may not yet have SimulationManager's stage-event cache.
    # set_device() then changes tensor readback but misses the authored GPU scene.
    # Enforce the requested native engine on the actual composed physics scene.
    device_settings=configure_scene_device(stage,bootstrap_device,broadphase=bootstrap_broadphase)
    # Same native contact-report path as the existing GPU dataset runner.
    # GPU dynamics stays enabled; CPU readback is needed for collider callbacks.
    carb.settings.get_settings().set_bool('/physics/suppressReadback',a.contact_backend=='tensor' and a.device.startswith('cuda'))
    sim.set_backend('torch')
    if a.threads:
        get_physx_interface().set_thread_count(a.threads)
    if a.performance=='optimized':
        from omni.physx.bindings._physx import SETTING_UPDATE_TO_USD,SETTING_UPDATE_VELOCITIES_TO_USD
        carb.settings.get_settings().set_bool(SETTING_UPDATE_TO_USD,False)
        carb.settings.get_settings().set_bool(SETTING_UPDATE_VELOCITIES_TO_USD,False)
        if a.headless:
            # update_fabric=False does not disable the native output buffers.
            # Disable both only when no viewport consumes them; native tensor
            # poses/contact reports and saved state remain available.
            carb.settings.get_settings().set_bool('/physics/fabricUpdateTransformations',False)
            carb.settings.get_settings().set_bool('/physics/fabricUpdateVelocities',False)
    # Allocation only: prevent silent missed broad-phase pairs in this dense farm.
    scene_api=PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath('/World/PhysicsScene'))
    from motion_app import configure_gpu_pair_capacities
    pair_allocation=configure_gpu_pair_capacities(stage,a.gpu_pair_capacity)
    scene_api.CreateGpuMaxNumPartitionsAttr(a.gpu_partitions)
    scene_api.CreateSolverTypeAttr(a.solver)
    events=[];step_events=[];current_time=0.;phase='idle'
    initialization_capture=dict(enabled=resume_context is not None or route_start_context is not None,
        active=True,preload_applied=False,contact_batches=[],native_break_callbacks=[],
        timestamp_scope='Native SDK initialization counters; separate from trial current_time=0',
        initialization_dynamics_changed=False)
    def initialization_clock():
        # Manual SDK warmup simulate calls use timestamp0. Native counters may
        # therefore stay0; report their actual values without inventing frames.
        result=dict(trial_time_s=current_time,phase=phase,preload_applied=initialization_capture['preload_applied'],
            resolved_gravity_m_s2=UsdPhysics.Scene(stage.GetPrimAtPath('/World/PhysicsScene')).GetGravityMagnitudeAttr().Get())
        for name,method in [('sdk_physics_step_count','get_num_physics_steps'),
                            ('sdk_simulation_time_s','get_simulation_time'),('sdk_physics_dt_s','get_physics_dt')]:
            try:result[name]=getattr(sim,method)()
            except Exception as error:result[name+'_query_error']=type(error).__name__+': '+str(error)
        return result
    if a.buffer_contact_events and a.contact_backend!='callback':
        raise ValueError('Buffered contact evidence requires the native callback backend')
    if a.record_contact_frames and not a.buffer_contact_events:
        raise ValueError('Contact frame recording requires --buffer-contact-events')
    path_cache={}
    def path(v):
        if a.performance=='baseline':return str(PhysicsSchemaTools.intToSdfPath(v))
        if v not in path_cache:path_cache[v]=str(PhysicsSchemaTools.intToSdfPath(v))
        return path_cache[v]
    contact_guard=None
    if a.fruit_contact_limit_mm!=a.contact_limit_mm or a.fruit_fruit_limit_mm!=a.contact_limit_mm:
        from motion_contact_guard import FruitContactGuard
        tool_colliders=[str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath('/World/ActuatedTool'))
            if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is not False]
        contact_guard=FruitContactGuard(manifest,tool_colliders,fruit_limit_m=a.fruit_contact_limit_mm/1000,
            fruit_fruit_limit_m=a.fruit_fruit_limit_mm/1000,default_limit_m=a.contact_limit_mm/1000)
        for path_name in contact_guard.fruits:
            prim=stage.GetPrimAtPath(path_name)
            if not prim or not prim.HasAPI(UsdPhysics.CollisionAPI):raise ValueError('Missing fruit collider '+path_name)
        (a.output/'contact_tolerance.json').write_text(json.dumps(contact_guard.metadata(),indent=2))
    contact_buffer=None
    if a.buffer_contact_events:
        from motion_contact_buffer import BufferedContactRecorder
        contact_buffer=BufferedContactRecorder(path,a.hz,capture_frames=a.record_contact_frames,
            penetration_limit_m=a.contact_limit_mm/1000,
            pair_limit=None if contact_guard is None else contact_guard.limit,
            native_triplet_slice=a.native_contact_vector_copy)
    def contact(headers,data):
        contact_started=time.perf_counter()
        capture_initial=initialization_capture['enabled'] and initialization_capture['active']
        raw_start=contact_buffer.record_count if contact_buffer is not None else len(events)
        if contact_buffer is not None:
            contact_buffer.record(headers,data,t=current_time,phase=phase)
        else:
            for h in headers:
                left,right=path(h.collider0),path(h.collider1)
                for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                    c=data[i]
                    e=dict(t=current_time,phase=phase,a=left,b=right,
                           separation_m=float(c.separation),force_n=math.sqrt(sum(float(x)**2 for x in c.impulse))*a.hz,
                           impulse_ns=list(c.impulse))
                    events.append(e);step_events.append(e)
        if capture_initial:
            initialization_capture['contact_batches'].append(dict(
                callback_index=len(initialization_capture['contact_batches']),
                raw_record_start=raw_start,
                raw_record_end=contact_buffer.record_count if contact_buffer is not None else len(events),
                **initialization_clock()))
        profile.total['contact_callback']+=time.perf_counter()-contact_started
        profile.count['contact_callback']+=1
    breaks=list(initial_inventory_breaks)
    def on_event(event):
        if event.type==int(SimulationEvent.JOINT_BREAK):
            record=dict(t=current_time,phase=phase,joint=str(PhysicsSchemaTools.decodeSdfPath(*event.payload['jointPath'])))
            breaks.append(record)
            if initialization_capture['enabled'] and initialization_capture['active']:
                initialization_capture['native_break_callbacks'].append(dict(
                    callback_index=len(initialization_capture['native_break_callbacks']),
                    event=dict(record),**initialization_clock()))
            print('[NATIVE BREAK]',round(current_time,4),phase,record['joint'],flush=True)
    leaf_armatures={f['joint']:manifest['compliant_foliage']['numerical_armature_kg_m2']
                    for f in manifest.get('compliant_foliage',{}).get('leaves',[])}
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            art=PhysxSchema.PhysxArticulationAPI.Apply(prim)
            art.CreateSolverPositionIterationCountAttr(a.position_iterations)
            art.CreateSolverVelocityIterationCountAttr(a.velocity_iterations)
        if prim.IsA(UsdPhysics.Joint) and str(prim.GetPath()).startswith('/World/ElasticTarget/Joints/'):
            PhysxSchema.PhysxJointAPI.Apply(prim).CreateArmatureAttr(leaf_armatures.get(str(prim.GetPath()),.0005))
    if a.num_envs>1 or a.replicate:
        from motion_replication import replicate_motion
        replication=replicate_motion(stage,manifest,a.num_envs,device=a.device,source_copy=a.replication_source_copy,
                                     native_replication=not a.replication_persist_layer_stack)
        envs=replication['environments']
        (a.output/'replication.json').write_text(json.dumps(replication,indent=2))
        if a.replication_update_boundary:app.update()
    materialized=None
    if a.replication_persist_layer_stack:
        from motion_materialize import materialize_replication_stage
        materialized=materialize_replication_stage(stage,a.output/'materialized_clone.usdc',replication,device=a.device)
        if not omni.usd.get_context().open_stage(materialized['file']):
            raise RuntimeError('Materialized clone stage open failed')
        for _ in range(5):app.update()
        stage=omni.usd.get_context().get_stage()
        actual_stage_metrics=require_motion_units(stage)
        # Stage OPENED resets SimulationManager caches/device/backend. Rebind
        # all native settings and USD handles; register callbacks only below.
        sim.set_default_physics_scene('/World/PhysicsScene')
        sim.set_device(a.device);sim.set_physics_dt(1/a.hz);sim.enable_fabric(True)
        device_settings=configure_scene_device(stage,a.device,broadphase=a.broadphase)
        sim.set_backend('torch')
        if a.threads:get_physx_interface().set_thread_count(a.threads)
        scene_api=PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath('/World/PhysicsScene'))
    # set_device(cuda) re-enables suppressReadback. Restore the requested
    # evidence backend after the final device registration, including reopen.
    from motion_app import configure_contact_readback
    contact_readback=configure_contact_readback(carb.settings.get_settings(),a.device,a.contact_backend)
    scene_ccd_before=bool(scene_api.GetEnableCCDAttr().Get())
    if a.scene_ccd:scene_api.CreateEnableCCDAttr(True)
    scene_ccd_enabled=bool(scene_api.GetEnableCCDAttr().Get())
    if a.scene_ccd and not scene_ccd_enabled:raise RuntimeError('Scene CCD request did not take effect')
    # Effective settings are recorded separately from the immutable assembly.
    (a.output/'runtime_settings.json').write_text(json.dumps(dict(hz=a.hz,solver=a.solver,armature_setup=a.armature_setup,
        gravity_neutral_warmup=a.gravity_neutral_warmup,scene_ccd_requested=a.scene_ccd,
        scene_ccd_before=scene_ccd_before,scene_ccd_enabled=scene_ccd_enabled,
        plant_armature=.0005,aggregate_pair_capacities=a.gpu_pair_capacity,initialization='armature authored before warmup',
        pair_allocation=pair_allocation,
        source_scene_sha256=sha(scene),native_scene=str(native_scene),native_scene_sha256=sha(native_scene),
        native_materialized_stage=a.native_materialized_stage,device=a.device,isolate_background=a.isolate_background,
        materialized_clone=materialized,
        replication_deferred_gpu_bootstrap=a.replication_defer_gpu_bootstrap,
        contact_readback=contact_readback,
        stage_metrics=actual_stage_metrics,
        native_scene_device=device_settings,
        disabled_background_colliders=isolated_count,performance=a.performance,threads=a.threads,
        display_fps=a.display_fps,realtime_cap=a.realtime,num_envs=a.num_envs,contact_backend=a.contact_backend,collision_filter=a.collision_filter,
        contact_limit_mm=a.contact_limit_mm,
        fruit_contact_limit_mm=a.fruit_contact_limit_mm,fruit_fruit_limit_mm=a.fruit_fruit_limit_mm,
        buffer_contact_events=a.buffer_contact_events,
        native_contact_vector_copy=a.native_contact_vector_copy,
        record_contact_frames=a.record_contact_frames,
        fabric_transform_output=carb.settings.get_settings().get('/physics/fabricUpdateTransformations'),
        fabric_velocity_output=carb.settings.get_settings().get('/physics/fabricUpdateVelocities'),
        plant_dynamics=a.plant_dynamics,zero_conveyor_diagnostic=a.zero_conveyor_diagnostic,
        position_iterations=a.position_iterations,velocity_iterations=a.velocity_iterations,
        gpu_partitions=a.gpu_partitions),indent=2))
    sub=get_physx_simulation_interface().subscribe_contact_report_events(contact) if a.contact_backend=='callback' else None
    break_sub=get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(on_event)
    if a.gravity_neutral_warmup:
        from motion_initialization import GravityNeutralWarmup
        warmup=GravityNeutralWarmup(stage,'/World/PhysicsScene');warmup.__enter__()
    if initialization_capture['enabled']:
        import inspect
        sdk_source=Path(inspect.getsourcefile(sim.initialize_physics)).resolve()
        initialization_capture.update(sdk_initialize_source=str(sdk_source),
            sdk_initialize_source_sha256=sha(sdk_source),before_initialize=initialization_clock())
    load=time.perf_counter();sim.initialize_physics()
    if sim.get_active_physics_engine()!='physx':raise RuntimeError('Motion trial requires native PhysX, not '+sim.get_active_physics_engine())
    effective_dt=float(sim.get_physics_dt())
    native_scenes=[]
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            api=PhysxSchema.PhysxSceneAPI(prim)
            native_scenes.append(dict(path=str(prim.GetPath()),
                enabled_gpu_dynamics=api.GetEnableGPUDynamicsAttr().Get(),
                broadphase_type=api.GetBroadphaseTypeAttr().Get(),
                steps_per_second=api.GetTimeStepsPerSecondAttr().Get(),
                solver=api.GetSolverTypeAttr().Get(),scene_ccd=api.GetEnableCCDAttr().Get()))
    (a.output/'native_scene_state.json').write_text(json.dumps(dict(
        physics_engine=sim.get_active_physics_engine(),
        requested_device=a.device,requested_broadphase=a.broadphase,requested_dt_s=1/a.hz,
        stage_metrics=actual_stage_metrics,
        effective_step_dt_s=effective_dt,scenes=native_scenes),indent=2))
    if not math.isclose(effective_dt,1/a.hz,rel_tol=0.,abs_tol=1e-12):
        raise RuntimeError('Native simulation timestep differs from requested timestep')
    if a.scene_ccd and any(s['scene_ccd'] is not True for s in native_scenes):raise RuntimeError('Native scene CCD state mismatch')
    if len(native_scenes)!=1 or native_scenes[0]['path']!='/World/PhysicsScene':
        raise RuntimeError('Unexpected native physics scene inventory: '+str(native_scenes))
    if native_scenes[0]['steps_per_second']!=a.hz:
        raise RuntimeError('Farm native scene Hz differs from requested Hz')
    if any(native_scenes[0][key]!=device_settings[key] for key in ('enabled_gpu_dynamics','broadphase_type')):
        raise RuntimeError('Native initialization changed requested dynamics/broadphase: '+str(native_scenes[0]))
    view=sim.get_physics_sim_view()
    def arr(v):
        if hasattr(v,'detach'):return v.detach().cpu().numpy()
        if hasattr(v,'numpy'):return v.numpy()
        return np.asarray(v)
    def tensor(x,dtype=torch.float32):return torch.as_tensor(x,dtype=dtype,device=sim.get_physics_sim_device())
    robot=view.create_articulation_view([e['robot'] for e in envs])
    plant=view.create_articulation_view([e['plant'] for e in envs])
    print('[MOTION] views',robot.count,plant.count,'backend',type(view),flush=True)
    if robot.count!=a.num_envs or plant.count!=a.num_envs:raise RuntimeError('Articulation identity mismatch')
    print('[MOTION] robot metadata',robot.shared_metatype.dof_names,flush=True)
    print('[MOTION] plant metadata',plant.shared_metatype.dof_names[:12],flush=True)
    print('[MOTION] plant links',plant.shared_metatype.link_names[:10],flush=True)
    print('[MOTION] native spring stiffness',arr(plant.get_dof_stiffnesses())[0,:9].tolist(),flush=True)
    if initialization_capture['enabled']:
        initialization_capture['after_initialize_before_preload']=dict(
            clock=initialization_clock(),plant_dof_names=list(plant.shared_metatype.dof_names),
            plant_q=arr(plant.get_dof_positions())[0].tolist(),
            plant_qvel=arr(plant.get_dof_velocities())[0].tolist(),
            preload_source='No runner preload command sent before native initialize_physics returned')
        (a.output/'native_initialization_timing.json').write_text(json.dumps(initialization_capture,indent=2))
    ids=tensor(list(range(a.num_envs)),torch.int32)
    q0=arr(robot.get_dof_positions()).copy()
    names=list(robot.shared_metatype.dof_names)
    railid=None
    if rail_destination is not None:
        railid=names.index(rail_destination['dof_name'])
        if resume_context is None and abs(float(q0[0,railid]))>.001:raise ValueError('Native initial rail joint is not at authored zero')
    elif resume_rail_origin is not None:
        railid=names.index(resume_rail_origin['dof_name'])
    rids=[names.index(n) for n in kin.names]
    if resume_context is None and 'initial_robot_q' in manifest and not np.allclose(q0[:,rids],initial_robot_q[None],atol=.005,rtol=0):
        raise RuntimeError('Native robot initialization does not match authored ready joint state: '+str(q0[:,rids]))
    jawids=[names.index('jaw_L'),names.index('jaw_R')]
    target_tensor=tensor(q0)
    robot.set_dof_position_targets(target_tensor,ids)
    if hasattr(plant,'set_dof_armatures'):
        from compliant_foliage import armature_values
        expected_armatures=armature_values(manifest,list(plant.shared_metatype.dof_names))
        armatures=plant.get_dof_armatures()
        if a.armature_setup=='refresh':
            plant.set_dof_armatures(tensor(np.broadcast_to(expected_armatures,armatures.shape).copy()),ids.cpu())
        elif not np.allclose(arr(armatures),expected_armatures[None],rtol=0.,atol=1e-10):
            raise RuntimeError('Authored native armature does not match per-joint configuration')
    # Reuse ElasticPlant's fixed authored-pose gravity preload, not continuous cancellation.
    jac=arr(plant.get_jacobians())[0];mass=arr(plant.get_masses())[0]
    preload=np.zeros(plant.max_dofs)
    for i,m in enumerate(mass):
        if i>0:preload-=jac[i-1,:3].T@np.array([0.,0.,-9.81*m])
    links=list(plant.shared_metatype.link_names)
    for f in manifest['plant']['fruits']:
        i=links.index(f['anchor'].rsplit('/',1)[1]);anchor=next(b for b in manifest['plant']['bodies'] if b['path']==f['anchor'])
        force=np.array([0.,0.,-9.81*f['mass']]);offset=np.array(f['center'])-anchor['rest_position']
        preload-=jac[i-1,:3].T@force+jac[i-1,3:].T@np.cross(offset,force)
    if resume_context is not None:preload=resume_context['arrays']['preload'].copy()
    preload_tensor=tensor(np.broadcast_to(preload,(a.num_envs,len(preload))).copy())
    plant.set_dof_actuation_forces(preload_tensor,ids)
    initialization_capture['preload_applied']=True
    if initialization_capture['enabled']:
        initialization_capture['preload_command']=dict(value=preload.tolist(),clock=initialization_clock(),
            source='Exact recorded runner set_dof_actuation_forces command after native initialization')
    from neighbor_plant_runtime import initialize as initialize_neighbors
    neighbor_handles,neighbor_records=initialize_neighbors(view,manifest,arr=arr,tensor=tensor,indices=ids)
    if neighbor_records:
        (a.output/'neighbor_initialization.json').write_text(json.dumps(neighbor_records,indent=2))
    if warmup is not None:
        warmup.restore()
        (a.output/'warmup.json').write_text(json.dumps(warmup.metadata,indent=2))
    if initialization_capture['enabled']:
        initialization_capture.update(after_preload_and_gravity_restore=initialization_clock(),active=False,
            manual_initialize_simulation_scope='Installed SDK initialize_physics performs two simulate(dt,0.0) calls; SDK counters may retain0, raw batches and their observed counters are recorded separately')
        (a.output/'native_initialization_timing.json').write_text(json.dumps(initialization_capture,indent=2))
    allpaths=[]
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):allpaths.append(str(prim.GetPath()))
    bodies=view.create_rigid_body_view(allpaths)
    body_paths=list(bodies.prim_paths)
    resume_validation=None
    if resume_context is not None:
        from resume_native_state import apply_resume
        resume_validation=apply_resume(resume_context,robot,plant,bodies,to_native=tensor,
            indices_to_native=lambda x:tensor(x,torch.int32),environment_indices=ids,
            refresh_articulations=view.update_articulations_kinematic)
        (a.output/'resume_native_validation.json').write_text(json.dumps(resume_validation,indent=2))
        if (resume_validation['max_position_error_m']>1e-4 or
            any(resume_validation[k]>1e-5 for k in ('robot_q_max_abs_error','plant_q_max_abs_error',
                'robot_qvel_max_abs_error','plant_qvel_max_abs_error'))):
            raise RuntimeError('Measured native resume state was not reproduced; see resume_native_validation.json')
        if route_restore['superseded_archive'] is not None:
            route_restore['superseded_archive'].update(selected_restore_executed=True,
                native_state_validation='resume_native_validation.json',
                native_validation_sha256=sha(a.output/'resume_native_validation.json'))
            (a.output/'route_start_superseded.json').write_text(json.dumps(route_restore['superseded_archive'],indent=2))
        q0=arr(robot.get_dof_positions()).copy();target_tensor=tensor(q0)
    route_start_validation=None
    if route_start_context is not None:
        from native_route_start import apply_route_start
        route_start_validation=apply_route_start(route_start_context,robot,plant,bodies,to_native=tensor,
            indices_to_native=lambda x:tensor(x,torch.int32),environment_indices=ids,
            refresh_articulations=view.update_articulations_kinematic)
        route_start_validation['numerical_restore_tolerances']=dict(position_m=1e-4,orientation_rad=1e-4,
            robot_q=1e-5,robot_qvel=1e-5,standalone_linear_velocity_m_s=1e-4,standalone_angular_velocity_rad_s=1e-4,
            articulated_link_velocity='diagnostic; q/qvel replay and fresh physics step separately checked')
        (a.output/'route_start_native_validation.json').write_text(json.dumps(route_start_validation,indent=2))
        if (route_start_validation['max_position_error_m']>1e-4 or
            route_start_validation['max_orientation_error_rad']>1e-4 or
            route_start_validation['standalone_max_linear_velocity_error_m_s']>1e-4 or
            route_start_validation['standalone_max_angular_velocity_error_rad_s']>1e-4 or
            any(route_start_validation[k]>1e-5 for k in ('robot_q_max_abs_error','robot_qvel_max_abs_error'))):
            raise RuntimeError('Measured native route start was not reproduced; see route_start_native_validation.json')
        q0=arr(robot.get_dof_positions()).copy();target_tensor=tensor(q0)
    initial=arr(bodies.get_transforms()).copy()
    if (resume_context is not None or route_start_context is not None) and len(breaks)!=len(initial_inventory_breaks):
        # Preserve this failure BEFORE geometry readers or contact-policy setup
        # can themselves raise. A native warmup break changes topology and may
        # not be removed by the measured old-robot state restore.
        warmup_events=[e for e in (contact_buffer.materialize() if contact_buffer is not None else events) if e['t']==0.]
        violations=[e for e in warmup_events if e['separation_m'] < -a.contact_limit_mm/1000 and
            any(x in e['a'] or x in e['b'] for x in ('/ActuatedTool/','/HarvestRobot/','/ElasticTarget/','/ElasticNeighbors/'))]
        (a.output/'pre_restore_initialization.json').write_text(json.dumps(dict(
            schema='s300-pre-restore-initialization-v1',state_scope='pre_restore',
            timestamp_scope='Native initialization warmup; before measured old state restore',
            contacts=warmup_events,violations=violations,native_break_events=breaks.copy(),
            evidence_is_restored_state=False,violations_are_retained=True),indent=2))
        np.savez_compressed(a.output/'invalid_initialization_state.npz',body_paths=np.array(body_paths),
            poses=initial,body_velocities=arr(bodies.get_velocities()).copy(),
            robot_dof_names=np.array(names),robot_q=arr(robot.get_dof_positions()).copy(),
            robot_qvel=arr(robot.get_dof_velocities()).copy(),
            plant_dof_names=np.array(list(plant.shared_metatype.dof_names)),
            plant_q=arr(plant.get_dof_positions()).copy(),plant_qvel=arr(plant.get_dof_velocities()).copy())
        (a.output/'initialization_failure.json').write_text(json.dumps(dict(
            schema='s300-native-window-initialization-failure-v1',physics_execution_started=False,
            valid=False,reason='new_native_joint_break_during_warmup',native_break_events=breaks.copy(),
            state_restore_verified=True,source_scene_sha256=sha(scene),
            actual_snapshot='invalid_initialization_state.npz',training_eligible=False,harvest_success=None),indent=2))
        try:
            from leaf_contact_geometry import CookedLeafContactGeometry
            from native_initialization_geometry import capture_initialization_geometry
            invalid_reader=CookedLeafContactGeometry(stage,manifest,body_paths,verify_fruit_contacts=True,
                verify_background_component_hulls=a.verify_background_leaf_enclosures,
                verify_structural_scene_queries=a.verify_structural_scene_query)
            invalid_geometry=capture_initialization_geometry(invalid_reader,warmup_events,initial,max_pairs=3)
        except Exception as error:
            invalid_geometry=dict(schema='s300-invalid-native-initialization-geometry-v1',inconclusive=True,
                error=type(error).__name__+': '+str(error),failure_overridden=False,training_eligible=False,harvest_success=None)
        (a.output/'invalid_initialization_geometry.json').write_text(json.dumps(invalid_geometry,indent=2))
        raise RuntimeError('Native warmup broke a new joint before measured restore; see pre_restore_initialization.json')
    if a.compiled_body_audit:
        from motion_compiled_audit import collect_compiled_body_audit
        reverse_mapping={new:old for old,new in envs[0]['mapping'].items()}
        compiled=collect_compiled_body_audit(stage,body_paths,path_mapping=reverse_mapping,
            articulation_views={'plant':plant,'robot':robot},device=a.device)
        (a.output/'compiled_body_audit.json').write_text(json.dumps(compiled,indent=2))
    toolids=[body_paths.index(e['tool']) for e in envs];targetids=[body_paths.index(e['target']) for e in envs]
    pulseids=[body_paths.index(a.pulse_body)] if a.pulse_body else targetids
    leaf_geometry=None
    if a.verify_leaf_contact_after_step or a.verify_harvest_contact_after_step:
        if not manifest.get('compliant_foliage'):raise ValueError('Leaf verification requires the explicit compliant foliage model')
        from leaf_contact_geometry import CookedLeafContactGeometry,save_imported_source as save_leaf_reader_source
        leaf_reader_source_provenance=save_leaf_reader_source(a.output)
        (a.output/'leaf_reader_source_provenance.json').write_text(json.dumps(leaf_reader_source_provenance,indent=2)+'\n')
        leaf_geometry=CookedLeafContactGeometry(stage,manifest,body_paths,verify_fruit_contacts=a.verify_harvest_contact_after_step,
            verify_background_component_hulls=a.verify_background_leaf_enclosures,
            verify_structural_scene_queries=a.verify_structural_scene_query)
        if a.near_trusses_metadata is not None and initial_sequence_group is not None:
            leaf_geometry.retarget_fruit_truss(manifest,initial_sequence_group['fruits'][0])
    if a.capture_leaf_colliders:
        if leaf_geometry is None:raise ValueError('Missing native leaf geometry reader')
        for path_name in a.capture_leaf_colliders:
            if path_name not in leaf_geometry.leaves|leaf_geometry.capsules:raise ValueError('Native cooking capture path is outside exact existing leaf/truss inventory')
            leaf_geometry.cooked(path_name)
        (a.output/'initial_leaf_cooked_geometry.json').write_text(json.dumps(leaf_geometry.geometry_metadata(),indent=2))
        np.savez_compressed(a.output/'initial_leaf_native_poses.npz',body_paths=np.array(body_paths),poses=initial)
    if a.capture_tool_colliders:
        from native_collider_capture import capture_tool_shapes
        capture_tool_shapes(stage,manifest,leaf_geometry,a.capture_tool_colliders,
            scene_path=scene,native_scene_path=native_scene,body_paths=body_paths,poses=initial,
            timestamp_s=0.,effective_hz=1/effective_dt,output=a.output/'native_tool_cooked_capture.json')
    from harvest_contact_scope import HarvestContactScope,collection_colliders_from_stage
    contact_scope=HarvestContactScope(manifest,collection_colliders_from_stage(stage,manifest),enabled=a.released_fruit_ground_diagnostic,stage=stage,gripper_support_enabled=a.released_fruit_gripper_support_diagnostic,same_truss_enabled=a.released_fruit_same_truss_diagnostic)
    from passive_collection_surfaces import save_imported_source as save_passive_surface_source
    passive_source_provenance=save_passive_surface_source(a.output)
    (a.output/'passive_surface_source_provenance.json').write_text(
        json.dumps(passive_source_provenance,indent=2)+'\n')
    foliage_conditioner=None
    if a.condition_foliage_seconds:
        if route_start_context is None:raise ValueError('Bounded foliage initialization currently requires an audited route-window state restore')
        from foliage_conditioning import FoliageConditioning,save_imported_source as save_conditioning_source
        save_conditioning_source(a.output)
        foliage_conditioner=FoliageConditioning(stage,manifest,body_paths,
            seconds=a.condition_foliage_seconds,hz=a.hz,mode=a.mode,
            source_context=dict(source_assembly=str(a.assembly.resolve()),scene_sha256=sha(scene),
                native_scene_sha256=sha(native_scene),source_plant=manifest['plant']['source_plant']))
        conditioning_begin_error=foliage_conditioner.begin(0.,initial,arr(bodies.get_velocities()).copy(),breaks)
        if conditioning_begin_error:raise RuntimeError(conditioning_begin_error)
    toolid=toolids[0];targetid=targetids[0]
    control_id=body_paths.index(belt_fixture['body_path']) if belt_fixture and a.num_envs==1 and not a.replicate else None
    tensor_contacts=None
    if a.contact_backend=='tensor':
        from motion_contacts import TensorContactReader
        tensor_contacts=TensorContactReader(view,body_paths,max_contact_data_count=max(16384,a.num_envs*16384))
        initial_tensor_events=tensor_contacts.read(t=0.,phase='initialization',dt=1/a.hz)
        events.extend(initial_tensor_events)
    initialization_events=[e for e in (contact_buffer.materialize() if contact_buffer is not None else events) if e['t']==0.]
    initialization_invalid=[e for e in initialization_events if e['separation_m'] < -a.contact_limit_mm/1000 and
        any(x in e['a'] or x in e['b'] for x in ('/ActuatedTool/','/HarvestRobot/','/ElasticTarget/','/ElasticNeighbors/'))]
    if tensor_contacts is not None and tensor_contacts.last_summary.get('raw_minimum_separation_m') is not None:
        if tensor_contacts.last_summary['raw_minimum_separation_m'] < -a.contact_limit_mm/1000 and not initialization_invalid:
            initialization_invalid=[dict(raw_sensor_summary=tensor_contacts.last_summary)]
    restored_initialization=resume_context is not None or route_start_context is not None
    trial_contact_start=0
    if restored_initialization:
        pre_restore=dict(schema='s300-pre-restore-initialization-v1',state_scope='pre_restore',
            timestamp_scope='Native initialization warmup contact buffers; solved before measured state restore',
            contacts=initialization_events,violations=initialization_invalid,native_break_events=breaks.copy(),
            tensor_contact_summary=None if tensor_contacts is None else tensor_contacts.last_summary.copy(),
            evidence_is_restored_state=False,violations_are_retained=True)
        (a.output/'pre_restore_initialization.json').write_text(json.dumps(pre_restore,indent=2))
        if len(breaks)!=len(initial_inventory_breaks):
            raise RuntimeError('Native warmup broke a new joint before measured restore; see pre_restore_initialization.json')
        trial_contact_start=contact_buffer.record_count if contact_buffer is not None else 0
        if contact_buffer is not None:contact_buffer.clear_step()
        events.clear();step_events.clear()
        initialization_invalid=[]
        initialization_validity=dict(valid=None,native_validity='pending_first_physics_step',
            physics_warmup_steps=2,timestamp_scope='Verified measured restore, before fresh solved physics step',
            state_restore_verified=True,pre_restore_evidence='pre_restore_initialization.json',
            reported_contact_points=0,violations=[],native_break_events=breaks)
    else:
        initialization_validity=dict(valid=not initialization_invalid,
            physics_warmup_steps=2,timestamp_scope='native initialize_physics before trial clock',
            reported_contact_points=len(initialization_events),violations=initialization_invalid,
            native_break_events=breaks)
    initialization_validity['contact_limit_mm']=a.contact_limit_mm
    (a.output/'initialization_validity.json').write_text(json.dumps(initialization_validity,indent=2))
    # Initial parity is checked at the measured saved station. The planning
    # kinematics may already be translated to a future rail destination.
    initial_fk_kin=station_kinematics(kin,resume_rail_origin) if resume_rail_origin is not None else kin
    actual=initial[toolid];expected=initial_fk_kin.fk(q0[0,rids])
    fk_error=float(np.linalg.norm(actual[:3]-expected[:3,3]))
    if fk_error>1e-4:raise RuntimeError(f'USD FK/native pose mismatch {fk_error}')
    (a.output/'native_metadata.json').write_text(json.dumps(dict(
        robot_dofs=names,plant_dofs=list(plant.shared_metatype.dof_names),body_paths=body_paths,
        plant_links=links,preload=preload.tolist(),fk_error_m=fk_error,
        model_sha256=sha(scene),physics_initialization_s=time.perf_counter()-load),indent=2))
    total=a.seconds or (sum(x[1] for x in phases)+2 if phases else 7.)
    cycle_start=2.;cycle_end=total;cycle_index=0;sequence=None;planning_paused_wall_s=0.
    if a.whole_truss:
        from truss_sequence import truss_fruits,inventory,select_group
        selected_fruits=sequence_fruits
        if any(f['path'] not in body_paths for f in selected_fruits):raise ValueError('Missing native truss fruit body')
        initial_group=initial_sequence_group['fruits'] if initial_sequence_group else manifest['grasp_reference']['fruits']
        if not set(initial_group)<=set(f['name'] for f in selected_fruits):raise ValueError('Grasp group spans other truss')
        attempts=initial_attempt_counts.copy()
        for name in initial_group:attempts[name]=attempts.get(name,0)+1
        cycle_end=2.+sum(p[1] for p in phases)
        total=a.seconds or (2.+(cycle_end-2.)*a.max_truss_cycles)
        sequence=dict(scope='SIM-GT remaining-fruit sequential native physics; no plant/robot/joint reset' if resume_context is None else 'SIM-GT measured cycle checkpoint continuation; new engine instance without contact-cache continuity',
            training_eligible=False,harvest_success=None,fruit_names=[f['name'] for f in selected_fruits],
            fruit_joints={f['name']:f['joint'] for f in selected_fruits},
            max_plan_attempts=a.max_truss_cycles,max_attempts_per_fruit=a.max_fruit_attempts,
            order=a.truss_order,initial_aim_source='authored fruit positions; subsequent aims use measured boundary state' if resume_context is None else 'measured native checkpoint remaining-fruit positions',
            resume_source=None if resume_context is None else resume_context['report'],
            prior_attempt_counts=initial_attempt_counts.copy(),attempt_counts=attempts.copy(),
            initial_axis=initial_axis,initial_plan_sha256=sha(a.output/'plan.json'),
            cycle_plans=[dict(index=0,fruits=initial_group,start_s=2.,planned_end_s=cycle_end,
                status='executing',plan='plan.json',measured_frame_index=0)],termination=None)
        if initial_sequence_group and initial_sequence_group.get('completed_pair_centering_recovery'):
            sequence['initial_centering_recovery']=initial_sequence_group['completed_pair_centering_recovery']
        if group_policy_selector is not None:
            sequence['group_policy']=group_policy
            sequence['cycle_plans'][0].update(initial_sequence_group)
        if a.near_trusses_metadata is not None:
            sequence['verified_truss_axes']=verified_truss_axes.copy()
            if initial_sequence_group and initial_sequence_group.get('near_scope_continuation'):
                sequence['initial_near_scope_selection']=initial_sequence_group['near_scope_continuation']
            sequence['cycle_plans'][0]['truss_key']=initial_sequence_group['truss_key']
        if a.later_grasp_depth_mm is not None:
            sequence['grasp_depth_schedule']=dict(base_depth_mm=a.grasp_depth_mm,
                later_depth_mm=a.later_grasp_depth_mm,prior_completed_native_cycles=prior_completed_native_cycles,
                activates_after='completed native physical cycle; never a rejected or stopped plan')
            sequence['cycle_plans'][0]['plan_parameters']=dict(grasp_depth_mm=initial_grasp_depth_mm,
                grasp_depth_schedule=initial_depth_evidence)
        (a.output/'truss_sequence.json').write_text(json.dumps(sequence,indent=2))
    trace=[initial.copy()];qtrace=[q0[0].copy()];times=[0.];phases_trace=['initial']
    command=[arr(robot.get_dof_position_targets())[0].copy()]
    stop_reason=('initialization_penetration_over_0_5mm' if a.contact_limit_mm==.5
                 else 'initialization_penetration_over_contact_policy') if initialization_invalid else None
    batch_qtrace=[q0.copy()]
    belt_commands=[(0.,0.)];force_commands=[(0.,0.,0.)]
    cycle_trace=[0]
    target=envs[0]['target'];last_phase=None
    belts=[PhysxSchema.PhysxSurfaceVelocityAPI(stage.GetPrimAtPath(x)) for e in envs for x in e['belts']]
    belt_attrs=[b.CreateSurfaceVelocityAttr() for b in belts];last_speed=None
    if a.performance=='optimized' and all(attr.Get()==Gf.Vec3f(0) for attr in belt_attrs):
        # Retain the already compiled zero command instead of sending a USD
        # notice for an unchanged surface property on the first trial step.
        last_speed=(0.,0.)
    # Use the existing broad rigid-body view. The native GPU one-body view
    # raised a misaligned-address error for this externally attached fruit.
    # A full persistent buffer preserves the same world force at exactly the
    # selected fruit rows and makes the native mapping unambiguous.
    force_tensor=tensor(np.zeros((len(body_paths),3))) if a.mode=='pulse' else None
    force_ids=tensor(list(range(len(body_paths))),torch.int32) if force_tensor is not None else None
    visuals=None
    if not a.headless:
        from motion_visuals import DeformationVisuals
        visuals=DeformationVisuals(stage,a.assembly,body_paths)
        view_camera='/World/RobotCamera'
        if a.preparation=='horizontal_ready' and a.mode=='harvest':
            from build_scene import camera
            pose=kin.fk(initial_robot_q);center=(pose[:3,3]+np.array(manifest['target']['center']))/2
            # Include the folded elbow as well as the gripper in the review view.
            center=center+np.array([0.,0.,.18])
            eye=center-pose[:3,0]*1.8+np.array([0.,0.,.30])
            view_camera='/World/ReadyMotionCamera';camera(stage,view_camera,tuple(eye),tuple(center))
        vu.get_active_viewport().set_active_camera(view_camera)
    print('[MOTION] physics ready',len(body_paths),'bodies',total,'sim seconds',flush=True)
    def save_native_route_state(destination, timestamp):
        from native_route_checkpoint import save_checkpoint
        return save_checkpoint(destination,assembly=a.assembly,scene_sha256=sha(scene),
            time_s=timestamp,body_paths=body_paths,body_poses_world=arr(bodies.get_transforms()).copy(),
            body_velocities_world=arr(bodies.get_velocities()).copy(),robot_dof_names=names,
            robot_q=arr(robot.get_dof_positions())[0].copy(),robot_qvel=arr(robot.get_dof_velocities())[0].copy(),
            robot_position_targets=arr(robot.get_dof_position_targets())[0].copy(),
            robot_velocity_targets=arr(robot.get_dof_velocity_targets())[0].copy(),
            plant_dof_names=list(plant.shared_metatype.dof_names),plant_q=arr(plant.get_dof_positions())[0].copy(),
            plant_qvel=arr(plant.get_dof_velocities())[0].copy(),preload=preload.copy(),breaks=breaks,
            manifest=manifest,sequence=sequence)
    native_profile=None
    if a.native_profile:
        from omni.physx import get_physx_benchmarks_interface
        native_profile=get_physx_benchmarks_interface();native_profile.enable_profile(True)
    if a.publish_live:
        from motion_live import MotionLiveWriter
        live=MotionLiveWriter(a.output,a.assembly,body_paths)
        live.publish(initial,.044+float(q0[0,jawids[1]]-q0[0,jawids[0]]),0.)
    if a.start_gate is not None:
        if a.device!='cpu':raise ValueError('Synchronized process pool is restricted to CPU')
        (a.output/'ready.json').write_text(json.dumps(dict(pid=os.getpid(),model_sha256=sha(scene),
            initial_valid=not initialization_invalid,ready_monotonic_s=time.perf_counter()),indent=2))
        waiting=time.perf_counter()
        while not a.start_gate.exists():
            if time.perf_counter()-waiting>a.gate_timeout:raise TimeoutError('CPU pool start barrier timed out')
            time.sleep(.05)
    started=time.perf_counter()
    rail_arrival_feedback=None
    cycle_open_feedback=None;cycle_final_open_feedback=None;entry_open_feedback=None;preparation_open_feedback={}
    last_display=-float('inf')
    last_publish=-float('inf')
    poses=initial.copy();q=q0.copy();current_time=0.;tick=-1
    for tick in range(0 if initialization_invalid else round(total*a.hz)):
        if sequence is not None and tick/a.hz>=cycle_end-1e-8:
            boundary=time.perf_counter()
            record=sequence['cycle_plans'][-1]
            record.update(status='motion_completed',end_s=current_time,end_frame_index=len(times)-1,
                          cycle_open_feedback=cycle_open_feedback,preparation_open_feedback=preparation_open_feedback,
                          cycle_final_open_feedback=cycle_final_open_feedback,
                          inventory_after=inventory(selected_fruits,breaks))
            print('[TRUSS] inventory',record['inventory_after'],flush=True)
            sequence['inventory']=inventory(selected_fruits,breaks)
            if a.checkpoint_cycle_states:
                sequence['checkpoint_capture_reason']='completed_cycle'
                sequence['checkpoint_physics_valid']=stop_reason is None
                save_native_route_state(a.output/'cycle_checkpoints'/f'{cycle_index:03d}',current_time)
                sequence.pop('checkpoint_capture_reason',None)
                sequence.pop('checkpoint_physics_valid',None)
            if not sequence['inventory']['remaining']:
                sequence['termination']='all_fruit_native_joints_detached';break
            if a.split_unproductive_pairs:
                from truss_sequence import unproductive_pairs
                physical_unproductive_pairs.update(unproductive_pairs(sequence['cycle_plans'],sequence['fruit_joints']))
                sequence['completed_unproductive_pairs']=[list(pair) for pair in sorted(physical_unproductive_pairs)]
            planned_next=False;rejected_groups=physical_unproductive_pairs.copy();rejected_fingerprints=set()
            if group_policy_selector is not None and a.later_grasp_depth_mm is not None:
                depth,_=grasp_depth_for_cycle(a.grasp_depth_mm,a.later_grasp_depth_mm,
                    prior_completed_native_cycles+completed_native_cycles(sequence['cycle_plans']))
                group_policy_selector.depth_m=depth/1000
            while len(sequence['cycle_plans'])<a.max_truss_cycles:
                group=sequence_selector(selected_fruits,breaks,body_paths,poses,attempts,
                    max_attempts=a.max_fruit_attempts,order=a.truss_order,
                    single_fruit_when_remaining=a.single_fruit_when_remaining,
                    rejected_groups=rejected_groups)
                selection_evidence(group)
                if group is None:break
                for name in group['fruits']:attempts[name]=attempts.get(name,0)+1
                cycle_index=len(sequence['cycle_plans'])
                folder=a.output/'cycles'/f'{cycle_index:03d}';folder.mkdir(parents=True,exist_ok=False)
                record=dict(index=cycle_index,**group,start_s=current_time,measured_frame_index=len(times)-1,
                            inventory_before=inventory(selected_fruits,breaks),status='planning')
                sequence['cycle_plans'].append(record)
                print('[TRUSS] planning',cycle_index,group['fruits'],flush=True)
                try:
                    center=np.array(group.get('aim_world_m',group['measured_centroid_world_m']))
                    center[2]+=manifest['grasp_reference'].get('height_offset_m',0.)
                    next_axis=None
                    if a.approach_yaw_mode=='truss-axis':
                        if a.near_trusses_metadata is not None:
                            next_axis=near_scope_truss_direction(group,selected_fruits,breaks,
                                body_paths,poses,planning_kin.world[:3,3],verified_truss_axes,
                                min_attached_fruits=a.truss_axis_min_fruits)
                        else:
                            next_axis=horizontal_truss_direction(selected_fruits,breaks,body_paths,poses,
                                last_axis['direction_world'],fallback_direction=last_axis['direction_world'],
                                min_attached_fruits=a.truss_axis_min_fruits)
                        record['truss_axis']=next_axis
                    from truss_sequence import planning_fingerprint
                    effective_depth_mm=a.grasp_depth_mm;depth_evidence=None
                    if a.later_grasp_depth_mm is not None:
                        if a.near_trusses_metadata is not None:
                            effective_depth_mm,depth_evidence=grasp_depth_for_truss_cycle(
                                a.grasp_depth_mm,a.later_grasp_depth_mm,
                                previous_cycle_records+sequence['cycle_plans'],group['truss_key'])
                        else:
                            effective_depth_mm,depth_evidence=grasp_depth_for_cycle(
                                a.grasp_depth_mm,a.later_grasp_depth_mm,
                                prior_completed_native_cycles+completed_native_cycles(sequence['cycle_plans']))
                        record['plan_parameters']=dict(grasp_depth_mm=effective_depth_mm,
                            grasp_depth_schedule=depth_evidence)
                    effective_height_mm=a.grasp_height_offset_mm;last_aim_evidence=None
                    if a.last_fruit_height_offset_mm is not None or a.last_fruit_depth_mm is not None:
                        effective_height_mm,effective_depth_mm,last_aim_evidence=selected_aim(
                            selected_fruits,breaks,group,height_mm=a.grasp_height_offset_mm,depth_mm=effective_depth_mm,
                            last_height_mm=a.last_fruit_height_offset_mm,last_depth_mm=a.last_fruit_depth_mm)
                        record.setdefault('plan_parameters',{}).update(last_remaining_fruit_aim=last_aim_evidence)
                    fingerprint_parameters={name:getattr(a,name) for name in ('preparation','horizontal_ready_connector','horizontal_ready_seed_index','horizontal_ready_normalize_turns','ready_lift_offset_mm','entry_retract_mm',
                         'grasp_height_offset_mm','grasp_lateral_offset_mm','grasp_depth_mm','preparation_gap_mm','preparation_opening','belt_seconds',
                         'close_seconds','belt_speed_m_s','sparse_entry_duration_scale')}
                    if a.later_grasp_depth_mm is not None:
                        fingerprint_parameters.update(grasp_depth_mm=effective_depth_mm,
                            later_grasp_depth_mm=a.later_grasp_depth_mm)
                    if last_aim_evidence is not None:
                        fingerprint_parameters.update(grasp_height_offset_mm=effective_height_mm,grasp_depth_mm=effective_depth_mm)
                    fingerprint=planning_fingerprint(group,initial_robot_q,next_axis,
                        fingerprint_parameters)
                    record['planning_fingerprint']=fingerprint
                    if fingerprint in rejected_fingerprints:
                        raise RuntimeError('Exact rejected planning query skipped at unchanged measured state')
                    new_phases,new_planning=make_plan(center-reference_center,next_axis,
                        grasp_depth_mm=effective_depth_mm,depth_evidence=depth_evidence,
                        grasp_height_mm=effective_height_mm,last_aim_evidence=last_aim_evidence)
                    if a.last_fruit_grasp_sway_mm:
                        new_phases,new_planning=apply_last_fruit_grasp_sway(planning_kin,new_phases,new_planning,
                            amplitude_m=a.last_fruit_grasp_sway_mm/1000,fruits=selected_fruits,
                            breaks=breaks,group=group,hz=a.hz)
                    (folder/'plan.json').write_text(json.dumps(new_planning,indent=2))
                    audit_external(a.assembly,folder/'self_collision_preflight.json',plan=folder/'plan.json',hz=a.hz,native_tool_cache=a.native_tool_collision_cache)
                    phases,planning=new_phases,new_planning
                    if leaf_geometry is not None and a.near_trusses_metadata is not None:
                        leaf_geometry.retarget_fruit_truss(manifest,group['fruits'][0])
                    if next_axis is not None:
                        last_axis=next_axis
                        if a.near_trusses_metadata is not None:
                            verified_truss_axes[next_axis['truss_key']]=next_axis
                            sequence['verified_truss_axes']=verified_truss_axes.copy()
                    cycle_start=current_time;cycle_end=cycle_start+sum(p[1] for p in phases)
                    cycle_open_feedback=None
                    cycle_final_open_feedback=None
                    entry_open_feedback=None
                    preparation_open_feedback={}
                    record.update(status='executing',plan=str((folder/'plan.json').relative_to(a.output)),planned_end_s=cycle_end)
                    planned_next=True;break
                except (RuntimeError,ValueError) as error:
                    rejected_groups.add(tuple(group['fruits']))
                    if record.get('planning_fingerprint'):rejected_fingerprints.add(record['planning_fingerprint'])
                    record.update(status='planning_rejected',error=str(error))
                    print('[TRUSS] plan rejected',str(error),flush=True)
            planning_paused_wall_s+=time.perf_counter()-boundary
            sequence['attempt_counts']=attempts
            if not planned_next:
                sequence['termination']=('group_'+group_policy_selector.last_decision['reason']
                    if group_policy_selector is not None and len(sequence['cycle_plans'])<a.max_truss_cycles
                    else 'attempt_budget_exhausted_remaining_fruit')
            (a.output/'truss_sequence.json').write_text(json.dumps(sequence,indent=2))
            if not planned_next:break
        control_started=time.perf_counter()
        t=tick/a.hz;phase='settle';qcommand=q0.copy();gap=.044;speed=(0.,0.)
        if resume_context is not None and t<2:
            # Native measured q includes the load-dependent drive error. Hold
            # the proven prior setpoint through settle; replacing it with q
            # would introduce another gravity sag before the planned connector.
            controller=resume_context.get('controller_state',{})
            prior_targets=controller.get('position_targets')
            if prior_targets is not None:qcommand[0]=prior_targets
        if route_start_context is not None and a.mode=='idle':
            qcommand[0]=route_start_context['controller_state']['position_targets']
            gap=.044+float(qcommand[0,jawids[1]]-qcommand[0,jawids[0]])
        pulse_force=np.zeros((a.num_envs,3))
        if a.mode=='pulse':
            phase='load' if 1<=t<2 else 'release' if t>=2 else 'settle'
            if 1<=t<2:pulse_force[:,0]=a.pulse_force_n
            if a.performance=='baseline' or np.any(pulse_force):
                force_tensor.zero_()
                force_tensor[pulseids,0]=a.pulse_force_n if 1<=t<2 else 0.
                bodies.apply_forces(force_tensor,force_ids,True)
        elif a.mode=='jaw' and t>1:
            phase='jaw_open_close';gap=.044+.016*np.sin((t-1)*np.pi/2)
            if 5<=t<6:phase='belt_infeed_unloaded';speed=(-.02,-.02)
            elif t>=6:phase='belt_opposed_unloaded';speed=(-.015,.015)
        elif a.mode=='belt_control':
            closed_gap=belt_fixture['command_closed_gap_m']
            if t<1:phase='control_close';gap=.044+float(closed_gap-.044)*t
            elif t<3:phase='control_infeed';gap=closed_gap;speed=(.02,.02)
            elif t<4:phase='control_opposed';gap=closed_gap;speed=(-.015,.015)
            else:phase='control_hold';gap=closed_gap
        elif (a.mode=='harvest' or rail_destination is not None) and t>=2:
            offset=t-cycle_start;previous_gap=.044
            for phase_index,(label,duration,qs,nextgap,lvel,rvel) in enumerate(phases):
                if offset<duration:
                    phase=label;u=offset/duration;u=u*u*(3-2*u)
                    f=u*(len(qs)-1);lo=min(int(f),len(qs)-2)
                    # Rail-only continuation holds the proven loaded lift/arm
                    # drive targets. Replacing them with measured q would drop
                    # the inherited gravity compensation when rail_move begins.
                    if not (route_start_context is not None and a.mode=='idle'):
                        qcommand[0,rids]=qs[lo]*(1-(f-lo))+qs[lo+1]*(f-lo)
                    if railid is not None:
                        qcommand[0,railid]=rail_command(planning['phases'][phase_index],u)
                    gap=previous_gap+(nextgap-previous_gap)*u;speed=(lvel,rvel)
                    if rail_destination is not None and label!='rail_move' and rail_arrival_feedback is None:
                        from rail_motion import rail_arrival_check
                        rail_arrival_feedback=rail_arrival_check(q[:,railid],qcommand[0,railid])
                        rail_arrival_feedback.update(t=t,phase=label,checked_before_motion_command=True)
                        if not rail_arrival_feedback['reached']:stop_reason='rail_arrival_not_reached'
                    opening=planning.get('preparation_opening')
                    if opening and label in opening['feedback_before_phases'] and label not in preparation_open_feedback:
                        feedback=cycle_opening_check(.044+q[:,jawids[1]]-q[:,jawids[0]],
                            required_gap_m=opening['required_gap_m'],tolerance_m=opening['tracking_tolerance_m'])
                        feedback.update(t=t,phase=label,checked_before_motion_command=True)
                        preparation_open_feedback[label]=feedback
                        if label=='approach':entry_open_feedback=feedback
                        if not feedback['reached']:stop_reason='entry_open_not_reached'
                    if label=='cycle_reclose' and cycle_open_feedback is None:
                        cycle_open_feedback=cycle_opening_check(.044+q[:,jawids[1]]-q[:,jawids[0]])
                        cycle_open_feedback['t']=t
                        if not cycle_open_feedback['reached']:
                            stop_reason='cycle_reopen_not_reached'
                    if (label=='withdraw' and (planning.get('belt_cycle') or {}).get('withdraw_requires_measured_open')
                            and cycle_final_open_feedback is None):
                        cycle_final_open_feedback=cycle_opening_check(.044+q[:,jawids[1]]-q[:,jawids[0]],
                            required_gap_m=planning['belt_cycle']['full_open_gap_m'],
                            tolerance_m=planning['belt_cycle']['opening_tracking_tolerance_m'])
                        cycle_final_open_feedback.update(t=t,phase=label,checked_before_motion_command=True)
                        if not cycle_final_open_feedback['reached']:stop_reason='cycle_final_open_not_reached'
                    if planning.get('belt_cycle') and a.belt_ramp_seconds and label in planning['belt_cycle']['phases']:
                        window=planning['belt_cycle']
                        factor=belt_speed_fraction(t-cycle_start-window['start_after_settle_s'],window['duration_s'],a.belt_ramp_seconds)
                        speed=(lvel*factor,rvel*factor)
                    elif label=='belt_spin' and a.belt_ramp_seconds:
                        factor=belt_speed_fraction(offset,duration,a.belt_ramp_seconds)
                        speed=(lvel*factor,rvel*factor)
                    break
                offset-=duration;previous_gap=nextgap
        if stop_reason in ('rail_arrival_not_reached','cycle_reopen_not_reached','cycle_final_open_not_reached','entry_open_not_reached'):break
        if a.num_envs>1:qcommand[1:]=qcommand[0]
        if a.belt_off_control:speed=(0.,0.)
        if not (route_start_context is not None and a.mode=='idle'):
            qcommand[:,jawids]=[-(gap-.044)/2,(gap-.044)/2]
        target_tensor.copy_(tensor(qcommand));robot.set_dof_position_targets(target_tensor,ids)
        if a.performance=='baseline' or speed!=last_speed:
            with profile.section('belt_usd_write'):
                for i,attr in enumerate(belt_attrs):attr.Set(Gf.Vec3f(0,float(speed[i%2]),0))
            last_speed=speed
        profile.total['control']+=time.perf_counter()-control_started;profile.count['control']+=1
        step_events.clear();current_time=(tick+1)/a.hz
        if contact_buffer is not None:contact_buffer.clear_step()
        if tick<2:profile.checkpoint(tick,'before_simulate')
        with profile.section('simulate'):
            sim.step(update_fabric=a.performance=='baseline')
        with profile.section('fetch'):
            get_physx_simulation_interface().fetch_results()
        if tensor_contacts is not None:
            with profile.section('tensor_contacts'):
                step_events=tensor_contacts.read(t=current_time,phase=phase,dt=1/a.hz);events.extend(step_events)
                if not tensor_contacts.last_summary.get('complete',False):
                    raise RuntimeError('Incomplete tensor contact mapping: '+str(tensor_contacts.last_summary))
        if contact_buffer is not None:step_events=contact_buffer.step_invalid_events()
        with profile.section('readback'):
            poses=arr(bodies.get_transforms()).copy();q=arr(robot.get_dof_positions()).copy()
        trace.append(poses);qtrace.append(q[0]);times.append(current_time);phases_trace.append(phase);command.append(qcommand[0])
        batch_qtrace.append(q.copy())
        belt_commands.append(speed);force_commands.append(tuple(pulse_force[0]))
        cycle_trace.append(cycle_index)
        if live is not None and time.perf_counter()-last_publish>=1/a.live_fps:
            with profile.section('live_publish'):
                live.publish(poses,.044+float(q[0,jawids[1]]-q[0,jawids[0]]),current_time)
            last_publish=time.perf_counter()
        if not np.isfinite(poses).all():stop_reason='nonfinite_state';break
        if phase!=last_phase:print('[MOTION]',round(t,3),phase,flush=True);last_phase=phase
        # Report all contacts; stop on robot or active-target gross penetration.
        invalid=([e for e in step_events if contact_guard.invalid(e)] if contact_guard else
                 [e for e in step_events if e['separation_m'] < -a.contact_limit_mm/1000 and
                  any(x in e['a'] or x in e['b'] for x in ('/ActuatedTool/','/HarvestRobot/','/ElasticTarget/','/ElasticNeighbors/'))])
        raw_conditioning_invalid=None
        if foliage_conditioner is not None:
            # Passive compliant leaves must remain in the unchanged strict
            # final guard even though the historical active-root guard did
            # not know their new exact namespace.
            invalid=[e for e in step_events if e['separation_m'] < -a.contact_limit_mm/1000 and
                (any(x in e['a'] or x in e['b'] for x in ('/ActuatedTool/','/HarvestRobot/','/ElasticTarget/','/ElasticNeighbors/')) or
                 e['a'] in foliage_conditioner.leaves or e['b'] in foliage_conditioner.leaves)]
            raw_conditioning_invalid=invalid.copy()
            conditioning_step=foliage_conditioner.before_geometry(current_time,invalid,breaks,
                poses,arr(bodies.get_velocities()).copy())
            invalid=conditioning_step['remaining_invalid']
            if conditioning_step['abort_reason']:
                stop_reason=conditioning_step['abort_reason'];break
        if leaf_geometry is not None and invalid:
            certificates={}
            worst_by_pair={}
            for e in invalid:
                key=tuple(sorted((e['a'],e['b'])))
                worst_by_pair[key]=min(worst_by_pair.get(key,0.),e['separation_m'])
            for key,separation in worst_by_pair.items():
                if leaf_geometry.eligible(*key):
                    certificates[key]=leaf_geometry.verify(*key,poses,current_time,separation,
                        limit_m=contact_guard.limit(*key) if contact_guard else a.contact_limit_mm/1000)['certified_within_limit']
            invalid=[e for e in invalid if not certificates.get(tuple(sorted((e['a'],e['b']))),False)]
        invalid=contact_scope.filter_invalid(invalid,breaks)
        if foliage_conditioner is not None:
            conditioning_step_error=foliage_conditioner.after_geometry(current_time,invalid)
            if conditioning_step_error:
                stop_reason=conditioning_step_error;print('[MOTION] conditioning stop',stop_reason,flush=True);break
        if restored_initialization and tick==0:
            initial_raw_min=tensor_contacts.last_summary.get('raw_minimum_separation_m') if tensor_contacts is not None else None
            initialization_validity.update(valid=not invalid and (initial_raw_min is None or initial_raw_min >= -a.contact_limit_mm/1000),
                native_validity='evaluated_first_physics_step',timestamp_scope='Fresh solved native step after measured restore',
                first_physics_timestamp_s=current_time,
                reported_contact_points=contact_buffer.step_contact_count if contact_buffer is not None else len(step_events),
                violations=invalid,raw_sensor_minimum_separation_m=initial_raw_min,
                contact_guard_unchanged=True,native_break_events=breaks.copy())
            if foliage_conditioner is not None:
                initialization_validity.update(valid=not raw_conditioning_invalid,
                    state_scope='initial_conditioning_state',operational_physics_valid=not raw_conditioning_invalid,
                    violations=raw_conditioning_invalid,raw_conditioning_invalid=raw_conditioning_invalid,
                    deferred_count=foliage_conditioner.step_records[-1]['deferred_count'],
                    conditional_continuation_is_not_clearance=True)
            (a.output/'initialization_validity.json').write_text(json.dumps(initialization_validity,indent=2))
        if invalid:
            stop_reason='penetration_over_contact_policy' if contact_guard or a.contact_limit_mm!=.5 else 'penetration_over_0_5mm'
            print('[MOTION] stop',invalid[:2],flush=True);break
        raw_min=tensor_contacts.last_summary.get('raw_minimum_separation_m') if tensor_contacts is not None else None
        if raw_min is not None and raw_min < -a.contact_limit_mm/1000:
            stop_reason='penetration_over_contact_policy' if a.contact_limit_mm!=.5 else 'penetration_over_0_5mm';print('[MOTION] stop raw sensor separation',tensor_contacts.last_summary,flush=True);break
        draw=(tick%max(1,round(a.hz/a.display_fps))==0) if a.performance=='baseline' else (time.perf_counter()-last_display>=1/a.display_fps)
        if visuals is not None and draw:
            profile.checkpoint(tick,'before_visual_update')
            with profile.section('visual_update'):
                visuals.update(poses,.044+float(q[0,jawids[1]]-q[0,jawids[0]]))
            if a.performance=='optimized':
                from omni.physxfabric import get_physx_fabric_interface
                with profile.section('fabric_display'):
                    get_physx_fabric_interface().update(current_time,1/a.hz)
            profile.checkpoint(tick,'before_app_update')
            with profile.section('app_update'):app.update()
            last_display=time.perf_counter()
        if a.realtime:
            remaining=current_time-(time.perf_counter()-started)
            if remaining>0:
                with profile.section('realtime_wait'):time.sleep(remaining)
        if tick%(a.hz*2)==0:
            elapsed=time.perf_counter()-started
            print('[MOTION] progress',round(current_time,2),'sim s /',round(elapsed,2),
                  'wall s /',round(current_time/max(elapsed,1e-9),3),'x realtime; target displacement mm',
                  np.linalg.norm(poses[targetid,:3]-initial[targetid,:3])*1000,flush=True)
            profile.checkpoint(tick,'progress')
    loop_finished=time.perf_counter();loop_wall=loop_finished-started
    conditioning_report=None
    if foliage_conditioner is not None:
        from foliage_conditioning import validate_report
        conditioning_report=foliage_conditioner.report()
        (a.output/'foliage_conditioning.json').write_text(json.dumps(conditioning_report,indent=2)+'\n')
        try:validate_report(conditioning_report)
        except ValueError as error:
            stop_reason=stop_reason or 'canonical_conditioned_start_not_verified'
            (a.output/'conditioning_rejection.json').write_text(json.dumps(dict(reason=str(error),
                stop_reason=stop_reason,no_harvest_credit=True),indent=2)+'\n')
        if stop_reason is None:
            # Retain the invalid beginning and expose the separate strict
            # terminal start. It is not an authored-start validity claim.
            (a.output/'conditioning_initialization_evidence.json').write_text(json.dumps(initialization_validity,indent=2)+'\n')
            initialization_validity=dict(valid=True,state_scope='canonical_conditioned_terminal_start',
                authored_start_evidence='conditioning_initialization_evidence.json',
                conditioning_report='foliage_conditioning.json',
                strict_window=conditioning_report['strict_window'],contact_guard_unchanged=True)
    if sequence is not None:
        sequence['inventory']=inventory(selected_fruits,breaks)
        if group_policy_selector is not None:
            from harvest_group_policy import harvest_group_outcome
            sequence['group_outcome']=harvest_group_outcome(group_policy,sequence['inventory'])
        sequence['attempt_counts']=attempts
        record=sequence['cycle_plans'][-1]
        if record['status']=='executing':
            reached_end=current_time>=cycle_end-1e-8
            record.update(status='motion_completed' if reached_end and stop_reason is None else 'stopped',
                end_s=current_time,end_frame_index=len(times)-1,stop_reason=stop_reason,
                cycle_open_feedback=cycle_open_feedback,preparation_open_feedback=preparation_open_feedback,
                cycle_final_open_feedback=cycle_final_open_feedback,
                inventory_after=sequence['inventory'])
        sequence['termination']=stop_reason or sequence['termination'] or (
            'all_fruit_native_joints_detached' if not sequence['inventory']['remaining'] else 'time_budget_exhausted_remaining_fruit')
        sequence.update(planning_paused_wall_s=planning_paused_wall_s,
            physical_loop_wall_s=loop_wall-planning_paused_wall_s,native_break_events=breaks)
        (a.output/'truss_sequence.json').write_text(json.dumps(sequence,indent=2))
    if contact_buffer is not None:
        events=contact_buffer.materialize(start=trial_contact_start)
        (a.output/'contact_buffer.json').write_text(json.dumps(contact_buffer.metadata(),indent=2))
    if live is not None:
        live.publish(poses,.044+float(q[0,jawids[1]]-q[0,jawids[0]]),current_time)
        live.close(stop_reason)
    if native_profile is not None:
        (a.output/'native_profile.json').write_text(json.dumps(native_profile.get_profile_stats(),indent=2,default=str))
        native_profile.enable_profile(False)
    profile.checkpoint(tick,'physics_complete')
    np.savez_compressed(a.output/'states.npz',time=np.array(times),poses=np.array(trace),
                        robot_q=np.array(qtrace),command=np.array(command),phase=np.array(phases_trace),body_paths=np.array(body_paths),
                        all_environment_robot_q=np.array(batch_qtrace),cycle_index=np.array(cycle_trace),
                        belt_surface_velocity_local_y_m_s=np.array(belt_commands),external_force_world_n=np.array(force_commands))
    if a.save_route_checkpoint:
        if conditioning_report is not None and stop_reason is None:
            from conditioned_start_checkpoint import conditioned_checkpoint_sequence
            sequence=conditioned_checkpoint_sequence(conditioning_report)
        elif sequence is not None and stop_reason is None:
            final_record=sequence['cycle_plans'][-1]
            if final_record.get('status')=='motion_completed' and abs(final_record['end_s']-current_time)<=1e-8:
                # A bounded single cycle can end exactly at the loop limit,
                # before the next iteration would save a boundary checkpoint.
                # Preserve this actual terminal capture without inventing an
                # extra physical cycle. Final summary/audit still gate reuse.
                sequence['checkpoint_capture_reason']='completed_cycle'
                sequence['checkpoint_physics_valid']=True
        save_native_route_state(a.output/'route_checkpoint',current_time)
    (a.output/'contacts.json').write_text(json.dumps(events))
    if leaf_geometry is not None:
        (a.output/'leaf_contact_resolution.json').write_text(json.dumps(leaf_geometry.records,indent=2))
        (a.output/'leaf_cooked_geometry.json').write_text(json.dumps(leaf_geometry.geometry_metadata()))
        (a.output/'solved_contact_policy.json').write_text(json.dumps(leaf_geometry.policy_metadata(),indent=2))
        query_helper=sys.modules.get('native_leaf_scene_query')
        if query_helper is not None:
            query_source_provenance=query_helper.save_imported_source(a.output)
            (a.output/'native_query_source_provenance.json').write_text(json.dumps(query_source_provenance,indent=2))
    (a.output/'break_events.json').write_text(json.dumps(breaks))
    (a.output/'contact_lifecycle_scope.json').write_text(json.dumps(contact_scope.summarize(breaks),indent=2))
    states=np.array(trace);qc=np.array(qtrace)
    target_events=[e for e in events if e['a']==target or e['b']==target or target+'/' in e['a'] or target+'/' in e['b']]
    summary=dict(mode=a.mode,scene=str(scene),model_sha256=sha(scene),training_eligible=False,
                 robot_self_collision_enabled=robot_self_collision_enabled,
                 execution_scope='entry_hold_return_only' if a.entry_only else a.mode,
                 harvest_profile=a.harvest_profile if a.mode=='harvest' else None,
                 belt_off_control=a.belt_off_control,cycle_open_feedback=cycle_open_feedback,
                 cycle_final_open_feedback=cycle_final_open_feedback,
                 entry_open_feedback=entry_open_feedback,preparation_open_feedback=preparation_open_feedback,
                 preparation_opening=a.preparation_opening,preparation_gap_mm=a.preparation_gap_mm,
                 contact_lifecycle_scope=contact_scope.summarize(breaks),
                 rail_destination=rail_destination,rail_arrival_feedback=rail_arrival_feedback,
                 native_resume=resume_validation,
                 native_route_start=route_start_validation,
                 route_start_source=None if route_start_context is None else route_start_context['report'],
                 route_idle_controller_hold=None if route_start_context is None or a.mode!='idle' else dict(
                     position_targets=route_start_context['controller_state']['position_targets'].tolist(),
                     measured_initial_q=q0[0].tolist(),
                     target_minus_measured_q=(route_start_context['controller_state']['position_targets']-q0[0]).tolist(),
                     provenance=route_start_context['controller_state']['provenance'],
                     measured_qvel_preserved=True,measured_q_is_not_drive_target=True),
                 resume_source=None if resume_context is None else resume_context['report'],
                 native_new_break_count=sum(e.get('native_new_event') is not False for e in breaks),
                 fruit_attachment=manifest.get('fruit_attachment'),
                 compliant_foliage=manifest.get('compliant_foliage'),pulse_body=a.pulse_body,
                 leaf_poststep_verification=a.verify_leaf_contact_after_step,
                 truss_sequence=sequence,planning_paused_wall_s=planning_paused_wall_s,
                 contact_limit_mm=a.contact_limit_mm,
                 fruit_contact_limit_mm=a.fruit_contact_limit_mm,fruit_fruit_limit_mm=a.fruit_fruit_limit_mm,
                 physics_steps=len(times)-1,simulated_s=times[-1] if times else 0,physics_wall_s=loop_wall,
                 loop_started_monotonic_s=started,loop_finished_monotonic_s=loop_finished,
                 save_wall_s=time.perf_counter()-started-loop_wall,performance=a.performance,threads=a.threads,
                 gui_enabled=not a.headless,display_fps_cap=a.display_fps if not a.headless else None,
                 num_envs=a.num_envs,aggregate_simulated_s=times[-1]*a.num_envs,
                 aggregate_sim_s_per_wall_s=times[-1]*a.num_envs/loop_wall,contact_backend=a.contact_backend,collision_filter=a.collision_filter,
                 contact_evidence_scope='exact collider impulse norm' if tensor_contacts is None else 'actor-level normal-only force; friction excluded; native A/B pending',
                 plant_dynamics=a.plant_dynamics,native_plant_dofs=plant.max_dofs,
                 zero_conveyor_diagnostic=a.zero_conveyor_diagnostic,
                 belt_control=None if control_id is None else dict(
                     transport_m=float(np.dot(states[-1,control_id,:3]-initial[control_id,:3],belt_fixture['transport_axis_world'])),
                     displacement_world_m=(states[-1,control_id,:3]-initial[control_id,:3]).tolist(),
                     fixture_only=True,harvest_success=False),
                 profile={name:dict(total_s=value,count=profile.count[name],max_s=profile.maximum[name]) for name,value in profile.total.items()},
                 completed=stop_reason is None,stop_reason=stop_reason,harvest_success=None,
                 initialization_valid=initialization_validity['valid'],
                 canonical_conditioned_start=conditioning_report is not None and stop_reason is None,
                 foliage_conditioning_report=None if conditioning_report is None else 'foliage_conditioning.json',
                 initialization_validity=initialization_validity,
                 native_target_detached=any(e['joint']==envs[0]['target_joint'] for e in breaks),
                 fixture_only=a.isolate_background,disabled_background_colliders=isolated_count,
                 native_break_events=breaks,physics_settings=dict(hz=a.hz,solver=a.solver,armature=.0005,device=a.device,scene_ccd=scene_ccd_enabled,
                     position_iterations=a.position_iterations,velocity_iterations=a.velocity_iterations,
                     gpu_partitions=a.gpu_partitions,
                     gpu_found_lost_aggregate_capacity=a.gpu_pair_capacity,gpu_total_aggregate_capacity=a.gpu_pair_capacity),
                 jaw_actual_gap_mm=(44+1000*(qc[:,jawids[1]]-qc[:,jawids[0]])).tolist(),
                 target_displacement_max_m=float(np.linalg.norm(states[:,targetid,:3]-initial[targetid,:3],axis=1).max()),
                 target_force_max_n=max((e['force_n'] for e in target_events),default=0),
                 force_metric='maximum norm of one contact point impulse / physics dt; not summed force or damage limit',
                 target_displacement_final_m=float(np.linalg.norm(states[-1,targetid,:3]-initial[targetid,:3])),
                 per_environment=[dict(index=i,target=e['target'],
                     target_displacement_max_m=float(np.linalg.norm(states[:,targetids[i],:3]-initial[targetids[i],:3],axis=1).max()),
                     target_displacement_final_m=float(np.linalg.norm(states[-1,targetids[i],:3]-initial[targetids[i],:3]))) for i,e in enumerate(envs)],
                 robot_tracking_max_m_or_rad=np.max(np.abs(qc-np.array(command)),axis=0).tolist(),
                 contact_events=len(events),min_contact_separation_m=min((e['separation_m'] for e in events),default=None),
                 limitations=['Not a validated grasp/detach detector','No real fruit damage calibration',
                              'Motion diagnostic; dynamic belt-loop deformation not modeled'])
    if manifest.get('plant_interaction_variant'):
        summary['plant_interaction_variant']=manifest['plant_interaction_variant']
    if neighbor_records:
        for (neighbor_view,_),record in zip(neighbor_handles,neighbor_records):
            record['final_q']=arr(neighbor_view.get_dof_positions())[0].tolist()
            record['final_qvel']=arr(neighbor_view.get_dof_velocities())[0].tolist()
        summary['elastic_neighbors']=neighbor_records
        summary['native_total_plant_dofs']=plant.max_dofs+sum(x['dof_count'] for x in neighbor_records)
        summary['neighbor_checkpoint_continuation_supported']=False
        (a.output/'neighbor_dynamics.json').write_text(json.dumps(neighbor_records,indent=2))
    if sequence is not None and a.whole_truss:
        from selected_cycle_evidence import build_selected_cycle_evidence
        selected_evidence=build_selected_cycle_evidence(
            manifest,sequence,breaks,body_paths,states,np.asarray(times),events,
            legacy_summary=summary,measured_phases=np.asarray(phases_trace))
        summary['selected_cycle_evidence']=selected_evidence
        (a.output/'selected_cycle_evidence.json').write_text(
            json.dumps(selected_evidence,ensure_ascii=False,indent=2)+'\n')
    if contact_guard:
        summary['contact_guard']=contact_guard.summarize(events,a.hz)
        (a.output/'contact_tolerance_evidence.json').write_text(json.dumps(summary['contact_guard'],indent=2))
    (a.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    if a.mode=='harvest' or a.rail_to_y is not None or foliage_conditioner is not None:
        from motion_self_collision import audit_external
        print('[MOTION] checking every stored actual arm/tool pose',flush=True)
        try:
            audit_external(a.assembly,a.output/'self_collision_actual.json',states=a.output/'states.npz',hz=a.hz,native_tool_cache=a.native_tool_collision_cache)
            summary['actual_self_collision_audit_passed']=True
        except Exception:
            summary.update(actual_self_collision_audit_passed=False,completed=False,
                           stop_reason=stop_reason or 'actual_self_collision_audit_failed')
            (a.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
            raise
        (a.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    if conditioning_report is not None and summary['canonical_conditioned_start'] and summary['completed']:
        from conditioned_start_checkpoint import write_conditioned_start_marker
        write_conditioned_start_marker(a.output/'route_checkpoint',a.output/'foliage_conditioning.json',
            a.output/'summary.json',native_stage_path=a.output/'native_stage_selection.json')
    print('[MOTION] saved',a.output,stop_reason,flush=True)
except Exception:
    exit_code=1;error=traceback.format_exc();print(error,flush=True)
    (a.output/'runtime_error.json').write_text(json.dumps(dict(error=error,elapsed_s=time.perf_counter()-start_wall),indent=2))
finally:
    if warmup is not None:warmup.restore()
    if live is not None and not live.closed:live.close('runtime_error')
    app.close(exit_code=exit_code)
