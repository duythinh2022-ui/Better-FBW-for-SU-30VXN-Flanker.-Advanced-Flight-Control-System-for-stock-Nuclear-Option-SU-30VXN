import sys, re
for f in sys.argv[1:]:
    tot = {}; states = set(); kern = set(); cst = set(); n = 0
    for l in open(f):
        n += 1
        for k, v in re.findall(r'(\w+)=(-?\d+)\b(?![.\w])', l):
            if k in ('solves', 'checked', 'identical', 'mismatches', 'fallbacks', 'grid', 'gridFallbacks', 'cpl', 'cplFallbacks', 'cplChecked', 'cplIdentical', 'cplMismatches'): tot[k] = tot.get(k, 0) + int(v)
        m = re.search(r'state=(.*?) grid=', l); states.add(m.group(1) if m else '?')
        m = re.search(r'kernels=(.*?) sweep=', l); kern.add(m.group(1) if m else '?')
        m = re.search(r'cplState=(.*)$', l); cst.add(m.group(1).strip() if m else '-')
    g = tot.get
    print(f"{f.split('/')[-1]:24s} runs {n:4d} | solves {g('solves',0):,} checked {g('checked',0):,} identical {g('identical',0):,} mism {g('mismatches',0)} fb {g('fallbacks',0)} | kernel {g('grid',0):,} fb {g('gridFallbacks',0)} | LP native {g('cpl',0):,} fb {g('cplFallbacks',0)} checked {g('cplChecked',0):,} identical {g('cplIdentical',0):,} mism {g('cplMismatches',0)} | {sorted(states)} {sorted(kern)} {sorted(cst)}")
