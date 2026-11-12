# compare two bench runs (tags) of one jet: every per-case CSV, column by column as text, except the wall-clock timing
# columns (fcsUs, estWaitUs, estThr, t*Us, sg*, heapB, gcRan, swMask), plus the suite tables (first line = tag, skipped)
# usage: csvcmp.py <bench out dir> <tagA> <tagB>
import os, sys, re, glob, csv
out, A, B = sys.argv[1:4]
TIME = re.compile(r'^(fcsUs|estWaitUs|estThr|t[A-Z]\w*Us|sg[A-Z]\w*|heapB|gcRan|swMask)$')
def files(tag):
    r = {}
    for root, dirs, fs in os.walk(out):
        parts = root.split(os.sep)
        if tag not in parts: continue
        k = parts.index(tag)
        for f in fs:
            if f.endswith('.csv'):
                key = os.sep.join(parts[:k] + ['<T>'] + parts[k+1:] + [f])
                r[key] = os.path.join(root, f)
    return r
fa, fb = files(A), files(B)
same = diff = 0; bad = []
for key in sorted(set(fa) | set(fb)):
    if key not in fa or key not in fb:
        bad.append((key, 'missing in ' + (A if key not in fa else B))); diff += 1; continue
    ra = list(csv.reader(open(fa[key]))); rb = list(csv.reader(open(fb[key])))
    if not ra and not rb:   # a case that failed (no rows) in both runs
        same += 1; continue
    if not ra or not rb or ra[0] != rb[0] and [c for c in ra[0] if not TIME.match(c)] != [c for c in rb[0] if not TIME.match(c)]:
        bad.append((key, 'header differs')); diff += 1; continue
    ka = [i for i, c in enumerate(ra[0]) if not TIME.match(c)]; kb = [i for i, c in enumerate(rb[0]) if not TIME.match(c)]
    if len(ra) != len(rb):
        bad.append((key, f'rows {len(ra)} vs {len(rb)}')); diff += 1; continue
    ok = True
    for n, (x, y) in enumerate(zip(ra, rb)):
        if [x[i] for i in ka if i < len(x)] != [y[i] for i in kb if i < len(y)]:
            cols = [ra[0][i] for i, j in zip(ka, kb) if i < len(x) and j < len(y) and x[i] != y[j]]
            bad.append((key, f'row {n}: ' + ','.join(cols[:8]))); ok = False; break
    if ok: same += 1
    else: diff += 1
tables = 0; tdiff = []
for f in sorted(glob.glob(os.path.join(out, f'out_*_{A}.txt'))):
    g = f[:-len(A) - 4] + B + '.txt'
    if not os.path.exists(g): tdiff.append(os.path.basename(f) + ' (no ' + B + ')'); continue
    # the tables print the tag (and the Name=value overrides) on a line of its own, and some in their header: not results
    def norm(text, tag):
        out = []
        for l in text.split('\n'):
            l = re.sub(r'(?<![\w.])' + re.escape(tag) + r'(?![\w.])', 'TAG', l)
            l = re.sub(r' in /\S+:line \d+', ' in <src>', l)   # a failing case's stack trace: the bench's source path and line
            if re.match(r'^(== )?TAG\b', l): continue
            if 'TAG' in l:   # a header naming the run: drop its Name=value overrides
                l = re.sub(r'\s+', ' ', re.sub(r'(?<=TAG)((\s+\w+=\S+)+)', '', l))
            out.append(l)
        return out
    a = norm(open(f).read(), A); b = norm(open(g).read(), B)
    if a == b: tables += 1
    else: tdiff.append(os.path.basename(g))
print(f'{A} vs {B}: per-case CSVs identical {same} / {same + diff}; suite tables identical {tables} / {tables + len(tdiff)}')
for k, why in bad[:20]: print('  DIFF', k, why)
for t in tdiff: print('  TABLE DIFF', t)
