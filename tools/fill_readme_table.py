#!/usr/bin/env python3
"""Fill the README results table from a batch metrics.json (single number source)."""
import json, re, sys, os

def main(batch):
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(root, 'results', batch, 'metrics.json'), encoding='utf-8') as f:
        J = json.load(f)
    rows = []
    for c in J['core']:
        m = c['metrics']
        est = 'N/A' if c['mode'] == 'ideal' else '%.3f/%.2f/%.2f' % tuple(m['est_rms_pvd'])
        rows.append('| %s | %s | %.3f | %.3f | %.3f | %s | %.1f | %.2f |' % (
            c['mode'], c['case_id'].replace('ideal_', '').replace('est_', '').replace('_', ' ')
            .replace('rtk lost', 'RTK degraded 5 s').replace('uncert Jplus', 'uncert. J +20%')
            .replace('uncert Jminus', 'uncert. J −20%'),
            m['pos_rms'], m['pos_max'], m['hov_off_norm'], est,
            m['tilt_max_deg'], m['effort_rms']))
    table = ('| Mode | Scenario | RMS 3-D error [m] | Max 3-D error [m] | Hover offset ‖b‖ [m] '
             '| est. p/v/d RMS | Max tilt | effort U_RMS |\n'
             '|:-----|:---------|-----:|-----:|-----:|:----:|-----:|-----:|\n' + '\n'.join(rows))
    p = os.path.join(root, 'README.md')
    src = open(p, encoding='utf-8').read()
    start = src.index('| Mode | Scenario |')
    end = src.index('\n\n*', start)
    src = src[:start] + table + src[end:]
    src = src.replace('<!-- RESULTS_TABLE_PLACEHOLDER: filled from results/<batch>/metrics.json by P5 -->',
                      '<!-- filled from results/%s/metrics.json -->' % batch)
    open(p, 'w', encoding='utf-8', newline='\n').write(src)
    print('README table filled from', batch)

if __name__ == '__main__':
    import sys
    main(sys.argv[1] if len(sys.argv) > 1 else '')
