"""Plot legacy pose and staged trajectory results without running Isaac."""


def plot_results(root, rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    staged = any(r['parameters'].get('trajectory_mode')=='staged6d' for r in rows)
    xkey='approach_azimuth_deg' if staged else 'azimuth_deg'
    ykey='insertion_distance_m' if staged else 'elevation_deg'
    scale=1000 if staged else 1
    fig,ax=plt.subplots(figsize=(7,5))
    for label in sorted({r['result'] for r in rows}):
        chosen=[r for r in rows if r['result']==label]
        ax.scatter([r['parameters'][xkey] for r in chosen],
                   [r['parameters'][ykey]*scale for r in chosen],label=label,s=30)
    ax.set(xlabel='Approach azimuth (deg)',ylabel='Insertion distance (mm)' if staged else 'Approach elevation (deg)')
    if rows: ax.legend()
    fig.tight_layout();fig.savefig(root/'candidate_results.png',dpi=150);plt.close(fig)
