"""Local desktop monitor: real training logs plus explicitly labelled 2D replay."""
import argparse
import json
import math
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk
import numpy as np
import yaml
from osp_mission.fast_sim import SearchEnv
from osp_mission.policy import BASELINE, load_policy


class Dashboard:
    def __init__(self,root,run,scenarios):
        self.root=root; self.runs_root=Path(run).resolve().parent; self.run=Path(run); self.scenarios=Path(scenarios)
        self.frames=queue.Queue(maxsize=2); self.running=True; self.latest=None
        root.title('OSP Training Live | 2D policy search + Gazebo verification')
        root.geometry('940x1000+970+30'); root.configure(bg='#101a2c')
        self.title=tk.Label(root,text='POLICY TRAINING · LIVE',font=('Sans',20,'bold'),fg='#eef5ff',bg='#101a2c'); self.title.pack(anchor='w',padx=20,pady=(16,4))
        tk.Label(root,text='HIDDEN ArUco • obstacle back faces • Train 120 / Val 32 / Test 48',fg='#9daec5',bg='#101a2c',font=('Sans',11)).pack(anchor='w',padx=20)
        self.status=tk.Label(root,text='Loading training metrics…',font=('Sans',12),fg='#5ee3ba',bg='#101a2c',justify='left'); self.status.pack(anchor='w',padx=20,pady=12)
        self.chart=tk.Canvas(root,height=170,bg='#17243b',highlightthickness=0); self.chart.pack(fill='x',padx=20)
        self.details=tk.Label(root,text='',fg='#d2deed',bg='#101a2c',font=('Sans',11),justify='left'); self.details.pack(anchor='w',padx=20,pady=10)
        tk.Label(root,text='2D SENSOR REPLAY — latest checkpoint, NOT Gazebo or a training worker',fg='#ffd180',bg='#101a2c',font=('Sans',10,'bold')).pack(anchor='w',padx=20)
        self.replay=tk.Canvas(root,height=405,bg='#07101f',highlightthickness=0); self.replay.pack(fill='both',expand=True,padx=20,pady=8)
        self.replay_stats=tk.Label(root,text='Waiting for replay…',fg='#a8c6f0',bg='#101a2c',font=('Sans',11)); self.replay_stats.pack(anchor='w',padx=20,pady=4)
        tk.Label(root,text='600 s simulated deadline | Nav2 handles physical motion and collision avoidance\nFast simulator metrics do not establish Gazebo / real-robot performance.',fg='#9daec5',bg='#101a2c',justify='left').pack(anchor='w',padx=20,pady=(4,16))
        root.protocol('WM_DELETE_WINDOW',self.close)
        threading.Thread(target=self.replay_worker,daemon=True).start(); self.refresh()

    def close(self): self.running=False; self.root.destroy()
    def put(self,item):
        try: self.frames.put_nowait(item)
        except queue.Full:
            try: self.frames.get_nowait()
            except queue.Empty: pass
            self.frames.put_nowait(item)
    def replay_worker(self):
        seed_index=0
        while self.running:
            try:
                files=sorted((self.scenarios/'validation').glob('*.yaml')); cfg=yaml.safe_load(files[seed_index%len(files)].read_text()); seed_index+=1
                path=self.run/'best_policy.json'; weights=load_policy(path) if path.exists() else BASELINE.copy()
                env=SearchEnv(cfg,seed=cfg['seed']+123456)
                trace=[(env.x,env.y)]
                def frame():
                    self.put((env.map.copy(),env.x,env.y,env.yaw,[(v[0],v[1]) for v in env.trace],env.elapsed,len(env.found),cfg['seed'],len(cfg.get('obstacles',[])),env.res))
                frame(); env.rotate(2*math.pi)
                while env.elapsed<600 and len(env.found)<3 and env.steps<160 and self.running:
                    env.step(weights); trace.append((env.x,env.y)); frame(); time.sleep(.25)
                time.sleep(1)
            except Exception as exc:
                self.put(str(exc)); time.sleep(2)

    def refresh(self):
        if not self.running: return
        try:
            live=self.runs_root/'live_state.json'
            if live.exists():
                state=json.loads(live.read_text())
                if state.get('run'):
                    self.run=Path(state['run'])
                    if (self.run/'scenarios').exists(): self.scenarios=self.run/'scenarios'
                    self.title.config(text='POLICY TRAINING · LIVE   /   ROUND '+str(state.get('round',0)))
            history_path=self.run/'history.json'; history=json.loads(history_path.read_text()) if history_path.exists() else []
            if history:
                last=history[-1]; stat=last['validation']; champion=json.loads((self.run/'best_policy.json').read_text()) if (self.run/'best_policy.json').exists() else {}
                best=champion.get('validation',stat)
                self.status.config(text=f'Generation {last["generation"]+1} / 24    ·    {last["training_episodes"]:,} training episodes\nValidation success {stat["success_rate"]:.1%}    ·    Best checkpoint {best["success_rate"]:.1%}')
                self.details.config(text=f'Latest: {stat["mean_found"]:.2f} / 3 targets   |   collisions {stat["collisions"]} / {stat["episodes"]} episodes\nWall time {last["wall_seconds"]/60:.1f} min   |   reward {stat["mean_reward"]:.1f}   |   updates every completed generation')
            else:
                self.status.config(text='Training running · evaluating initial policies / generation 1\nEpisode batches execute on CPU workers')
            report=self.run/'test_report.json'
            if report.exists():
                r=json.loads(report.read_text()); self.status.config(text=f'Run complete · {r["training_episodes"]:,} training episodes\nHeld-out success {r["learned"]["success_rate"]:.1%} | baseline {r["baseline"]["success_rate"]:.1%} | threshold passed: {r["passed"]}')
            self.draw_chart(history)
        except (OSError,ValueError,KeyError): pass
        try:
            while True: self.latest=self.frames.get_nowait()
        except queue.Empty: pass
        if self.latest and not isinstance(self.latest,str): self.draw_replay(self.latest)
        self.root.after(1200,self.refresh)

    def draw_chart(self,history):
        c=self.chart; c.delete('all'); w=max(c.winfo_width(),600); h=170
        c.create_text(15,14,text='Validation success rate (%)',anchor='w',fill='#c8d8ef')
        for val in [0,50,100]:
            y=145-val*1.05; c.create_line(45,y,w-15,y,fill='#30435e'); c.create_text(30,y,text=str(val),fill='#8fa5c2')
        pts=[]
        for i,row in enumerate(history): pts.extend([50+i*(w-75)/23,145-row['validation']['success_rate']*105])
        if len(pts)>=4: c.create_line(*pts,fill='#5ee3ba',width=3)
        for i in range(0,len(pts),2): c.create_oval(pts[i]-3,pts[i+1]-3,pts[i]+3,pts[i+1]+3,fill='#5ee3ba',outline='')

    def draw_replay(self,frame):
        data,x,y,yaw,trace,elapsed,found,seed,difficulty,res=frame
        c=self.replay; c.delete('all'); w=max(c.winfo_width(),600); h=max(c.winfo_height(),300)
        scale=min((w-24)/(data.shape[1]*res),(h-24)/(data.shape[0]*res)); left=(w-data.shape[1]*res*scale)/2; bottom=(h+data.shape[0]*res*scale)/2
        def xy(a,b): return left+a*scale,bottom-b*scale
        c.create_rectangle(*xy(0,0),*xy(data.shape[1]*res,data.shape[0]*res),fill='#263751',outline='')
        for row in range(data.shape[0]):
            # Run-length painting keeps Tk responsive.
            start=0
            for col in range(1,data.shape[1]+1):
                if col==data.shape[1] or data[row,col]!=data[row,start]:
                    val=data[row,start]
                    if val>=0: c.create_rectangle(*xy(start*res,row*res),*xy(col*res,(row+1)*res),fill='#d9e2ee' if val==0 else '#243040',outline='')
                    start=col
        coords=[]
        for a,b in trace: coords.extend(xy(a,b))
        if len(coords)>3: c.create_line(*coords,fill='#36b9ff',width=2)
        px,py=xy(x,y); c.create_oval(px-5,py-5,px+5,py+5,fill='#ffbd69',outline='')
        c.create_line(px,py,px+15*math.cos(yaw),py-15*math.sin(yaw),fill='#ffbd69',width=3)
        self.replay_stats.config(text=f'Seed {seed} · {difficulty} obstacles · {min(elapsed,600):.0f} / 600 simulated s · markers {found}/3')


def main():
    p=argparse.ArgumentParser(); p.add_argument('--run',default='training_hidden/initial'); p.add_argument('--scenarios',default='scenarios_hidden')
    a=p.parse_args(); root=tk.Tk(); Dashboard(root,a.run,a.scenarios); root.mainloop()

if __name__=='__main__': main()
