import sys, math
sys.argv=[sys.argv[0]]
exec(open('check_assembly.py').read().split('worst_hand =')[0])
for n, W in workers.items():
    print('== worker', n)
    rows=[]
    for f in range(0, n_frames, 15):
        t=f/C.FPS; J=world_joints(n,f); specs=C.hand_specs(W,t,ctx); p,h=W.root_at(t)
        errs=[]
        for side in 'LR':
            s=specs[side]
            if s is None or (isinstance(s,tuple) and s[0]=='blend'): errs.append('-'); continue
            palm=(J[side+'_Hand']+J[side+'_Mid1'])/2; errs.append('%.0f'%(palm-s[0]).GetLength())
            if side=='L': g=s[0]
        if any(e!='-' and float(e)>8 for e in errs):
            sh=J['L_Upperarm']; rows.append('t=%5.1f root(%4.0f,%4.0f) h=%4.0f errs %s shoulder y %.0f target %s'%(t,p[0],p[2],h,errs,sh[1],[round(v) for v in g] if errs[0]!='-' else ''))
    print('\n'.join(rows[:60]))
