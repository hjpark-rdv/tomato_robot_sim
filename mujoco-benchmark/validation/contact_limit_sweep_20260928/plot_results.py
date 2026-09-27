from pathlib import Path
import gzip,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent
fig,axs=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
for name,color in [('original_00_f5_d20','tab:blue'),('slow18_00_f5_d20','tab:orange')]:
 rows=json.loads(gzip.decompress((O/name/'authorized_contact_samples.json.gz').read_bytes()))
 t=np.array([r['time_s'] for r in rows]);f=np.array([r['contact_categories_private']['target_fruit_touch']['force_sum_N'] for r in rows]);d=np.array([r['target_displacement_m']*1000 for r in rows])
 hit=np.flatnonzero(f>0);start=max(0,hit[0]-2) if len(hit) else 0
 label='Original speed' if name.startswith('original') else '18x slower seat'
 axs[0].plot(t[start:]-t[start],d[start:],color=color,label=label)
 axs[1].plot(d[start:],f[start:],color=color,label=label)
 axs[0].scatter(t[-1]-t[start],d[-1],color=color,marker='x');axs[1].scatter(d[-1],f[-1],color=color,marker='x')
axs[0].set(xlabel='Time since near first fruit force (s)',ylabel='Fruit displacement from initial state (mm)')
axs[1].set(xlabel='Fruit displacement from initial state (mm)',ylabel='Private-forward fruit force magnitude sum (N)')
for a in axs:a.grid(alpha=.3);a.legend()
fig.suptitle('Same seating_00 joint path, limits 5 N / 20 mm; x = final recorded state')
fig.savefig(O/'force_displacement.png',dpi=160);fig.savefig(O/'force_displacement.svg')

# Normalize generated SVG whitespace for repository checks.
p=O/"force_displacement.svg"
p.write_text("\n".join(line.rstrip() for line in p.read_text().splitlines())+"\n")
