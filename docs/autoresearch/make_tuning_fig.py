import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

rows = [json.loads(l) for l in open('.auto/log.jsonl', encoding='utf-8')]
runs = [r for r in rows if r.get('type') == 'run']

x    = [r['run'] for r in runs]
rms  = [r['metric'] for r in runs]
st   = [r['status'] for r in runs]
tilt = [r.get('metrics', {}).get('tilt_ertk_deg') for r in runs]

COL = {'keep': '#059669', 'discard': '#dc2626', 'checks_failed': '#ea580c', 'noop': '#6b7280'}
LBL = {'keep': 'keep', 'discard': 'discard', 'checks_failed': 'tilt > 35 deg (rejected)', 'noop': 'rerun'}

fig, (a1, a2) = plt.subplots(1, 2, figsize=(12.5, 4.2), facecolor='w')

# --- panel 1: full run progression ---
best = []
b = 1e9
for v in rms:
    b = min(b, v)
    best.append(b)
a1.axhline(rms[0], color='#94a3b8', ls=':', lw=1)
a1.text(0.3, rms[0] + 0.0004, 'baseline 0.1369', fontsize=8, color='#64748b')
for s in COL:
    xs = [xx for xx, ss in zip(x, st) if ss == s]
    ys = [yy for yy, ss in zip(rms, st) if ss == s]
    a1.plot(xs, ys, 'o', ms=6, color=COL[s], label=LBL[s], zorder=3)
a1.plot(x, best, '-', color='#059669', lw=1.4, alpha=0.85, zorder=2)
a1.annotate('12-state\ntwo-time-scale KF', xy=(7, rms[6]), xytext=(5.1, 0.149),
            fontsize=8, ha='center',
            arrowprops=dict(arrowstyle='->', color='#475569', lw=0.8))
a1.annotate('soft re-acquisition\nramp', xy=(3, rms[2]), xytext=(2.0, 0.151),
            fontsize=8, ha='center',
            arrowprops=dict(arrowstyle='->', color='#475569', lw=0.8))
a1.annotate('qf sweep', xy=(10, rms[9]), xytext=(11.5, 0.122), fontsize=8, ha='center',
            arrowprops=dict(arrowstyle='->', color='#475569', lw=0.8))
a1.annotate('qs sweep', xy=(18, rms[17]), xytext=(18.3, 0.142), fontsize=8, ha='center',
            arrowprops=dict(arrowstyle='->', color='#475569', lw=0.8))
a1.set_xlabel('experiment run'); a1.set_ylabel('ertk RMS 3-D error [m]')
a1.set_title('(a) 20-run autoresearch progression', fontsize=10)
a1.legend(fontsize=8, loc='lower left', framealpha=0.9)
a1.grid(alpha=0.3); a1.set_ylim(0.118, 0.158); a1.set_xticks(range(2, 21, 2))

# --- panel 2: qf sweep + tilt cliff ---
qf  = [0.2, 0.5, 1.0, 2.0, 4.0, 5.0, 6.0, 8.0]
qr  = [rms[6], rms[7], rms[8], rms[9], rms[10], rms[14], rms[12], rms[11]]
qt  = [tilt[6], tilt[7], tilt[8], tilt[9], tilt[10], tilt[14], tilt[12], tilt[11]]
a2.plot(qf, qr, 'o-', color='#2563eb', lw=1.5, ms=6, label='ertk RMS [m]', zorder=3)
for xx, yy in zip(qf, qr):
    a2.annotate(f'{yy:.4f}', xy=(xx, yy), xytext=(0, 6), textcoords='offset points',
                fontsize=7, ha='center', color='#1e40af')
a2.set_xlabel('gust process-noise intensity  q_f'); a2.set_ylabel('ertk RMS [m]', color='#2563eb')
a2.tick_params(axis='y', labelcolor='#2563eb')
a2.set_xscale('log'); a2.set_xticks(qf); a2.set_xticklabels(['0.2', '0.5', '1', '2', '4', '5', '6', '8'])
a2.minorticks_off(); a2.set_ylim(0.118, 0.152)
a2b = a2.twinx()
a2b.axhspan(35, 52, color='#fee2e2', alpha=0.6, zorder=1)
a2b.text(1.6, 44, 'tilt limit 35 deg', fontsize=8, color='#b91c1c')
a2b.plot(qf, qt, 's--', color='#ea580c', lw=1.3, ms=5, label='max tilt [deg]', zorder=3)
a2b.axhline(35, color='#dc2626', ls='--', lw=1)
a2b.set_ylabel('max tilt [deg]', color='#ea580c'); a2b.tick_params(axis='y', labelcolor='#ea580c')
a2b.set_ylim(0, 52)
a2.set_title('(b) q_f sweep: RMS gain vs tilt cliff', fontsize=10)
a2.grid(alpha=0.3, which='major')

fig.suptitle('')
fig.tight_layout(rect=(0, 0, 1, 1))
fig.savefig('docs/tuning.png', dpi=140, facecolor='w')
print('saved docs/tuning.png')
