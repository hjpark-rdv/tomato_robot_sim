"""Keep existing active-physics validity separate from diagnostic scene metrics."""

def active_penetration_violations(rows, limit_m=.0005):
    return [r for r in rows if r['penetration_m'] > limit_m
            and set(r['categories']) & {'robot','active_plant'}]

def idle_validity(record):
    reasons=[]
    if not record['finite']:reasons.append('nonfinite_state')
    if any(record['warning_counts']):reasons.append('simulator_warning')
    violations=active_penetration_violations(record['worst_contacts'])
    if violations:reasons.append('robot_or_active_plant_penetration')
    return dict(passed=not reasons,reasons=reasons,violations=violations,
        displacement_is_diagnostic=True,penetration_limit_m=.0005)
