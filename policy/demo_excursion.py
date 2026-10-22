"""Recorded Newton excursion display using the real planner's metric.

No IK, planning or threshold enforcement. CARRY target boundaries use exactly
the deployment export's box-pose sampler; other phases use tool-pose slices.
Recorded joint interpolation differs from a live MoveIt Cartesian solution.
"""
import math
from pathlib import Path
import sys
import numpy as np
from demo_pose_sampling import sample_pose_indices

# Import the same ROS-free implementation used by the installed planner.
planning_source = str(Path(__file__).resolve().parents[1] / 'src/luggage_planning')
if planning_source not in sys.path:
    sys.path.insert(0, planning_source)
from luggage_planning.motion_excursion import cartesian_excursion


MOVING = {'APPROACH', 'PICK', 'LIFT', 'CARRY', 'PLACE', 'RETREAT', 'RETURN', 'ROTATE'}


class ExcursionDisplay:
    def __init__(self, data):
        self.frame = 0
        self.segments, self.phases = [], []
        self.by_frame = [None] * len(data['stage'])
        index = 0
        while index < len(data['stage']):
            stage, box = str(data['stage'][index]), str(data['box_id'][index])
            stop = index+1
            while stop < len(data['stage']) and str(data['stage'][stop]) == stage and str(data['box_id'][stop]) == box:
                stop += 1
            if stage in MOVING:
                start = index-1 if index and str(data['box_id'][index-1]) == box else index
                polyline = float(np.linalg.norm(np.diff(data['tool_pose'][start:stop,:3],axis=0),axis=1).sum())
                phase = cartesian_excursion([],list(data['joint_q'][start:stop]),polyline)
                self.phases.append(dict(stage=stage,box_id=box,start_frame=start,end_frame=stop-1,
                                        tcp_polyline_m=polyline,metric=phase))
                poses = data['box_pose'] if stage == 'CARRY' else data['tool_pose']
                targets = [index+i for i in sample_pose_indices(poses[index:stop])]
                if targets[-1] != stop-1:
                    targets.append(stop-1)
                previous = start
                for number, end in enumerate(targets):
                    if end <= previous:
                        continue
                    chord = float(np.linalg.norm(data['tool_pose'][end,:3]-data['tool_pose'][previous,:3]))
                    metric = cartesian_excursion([],list(data['joint_q'][previous:end+1]),chord)
                    record = dict(stage=stage,box_id=box,start_frame=previous,end_frame=end,
                                  metric=metric,phase_metric=phase,
                                  sampling='exported_box_via' if stage=='CARRY' else 'tool_pose_diagnostic_slice',
                                  segment_name='place_via_%d' % number if stage=='CARRY' else '%s slice %d' % (stage,number))
                    self.segments.append(record)
                    for frame in range(max(index,previous+1),end+1):
                        self.by_frame[frame]=record
                    if self.by_frame[index] is None:
                        self.by_frame[index]=record
                    previous=end
            index=stop
        metrics=[s['metric'] for s in self.segments if s['metric']]
        self.summary=dict(segments=len(metrics),over_3=sum(m['rad_per_m']>3 for m in metrics),
                          over_4=sum(m['rad_per_m']>4 for m in metrics),
                          maximum_rad_per_m=max((m['rad_per_m'] for m in metrics),default=0))

    def report(self):
        return dict(metric='max_joint_total_variation_rad / max(requested_TCP_chord_m, 0.05)',
                    scope='recorded simulation joints; not a live MoveIt trajectory',
                    carry_sampling='same box-pose target indices as hardware profile exporter',
                    other_sampling='diagnostic tool-pose slices, not hardware command boundaries',
                    summary=self.summary,phases=self.phases,segments=self.segments)

    def render(self, imgui):
        imgui.set_next_window_pos(imgui.ImVec2(20,30),imgui.Cond_.first_use_ever)
        imgui.set_next_window_size(imgui.ImVec2(530,330),imgui.Cond_.first_use_ever)
        if imgui.begin('Joint travel / TCP distance (SIM)'):
            imgui.text('Diagnostic only. 3 / 4 are reference lines, not stops.')
            imgui.text('Full replay peak: %.3f rad/m | >3: %d  >4: %d' % (
                self.summary['maximum_rad_per_m'],self.summary['over_3'],self.summary['over_4']))
            imgui.separator()
            record=self.by_frame[min(self.frame,len(self.by_frame)-1)]
            if record and record['metric']:
                m=record['metric']
                imgui.text('%s | %s | %s' % (record['box_id'],record['stage'],record['segment_name']))
                imgui.text('Current slice: %.3f rad/m   %s' % (m['rad_per_m'],m['joint']))
                imgui.text('Joint total travel: %.4f rad = %.1f deg' % (m['excursion_rad'],math.degrees(m['excursion_rad'])))
                imgui.text('TCP chord: %.2f cm | denominator: %.2f cm' % (100*m['path_len_m'],100*max(.05,m['path_len_m'])))
                imgui.text('Frames %d - %d | now %d' % (record['start_frame'],record['end_frame'],self.frame))
                imgui.text('Above 3: %s | Above 4: %s' % (m['rad_per_m']>3,m['rad_per_m']>4))
                if record['phase_metric']:
                    imgui.text('Whole phase / TCP polyline: %.3f rad/m' % record['phase_metric']['rad_per_m'])
            else:
                imgui.text('Stationary / waiting for Pick.')
            imgui.separator()
            imgui.text('CARRY slices match export target indices.')
            imgui.text('Other phases use diagnostic slices.')
            imgui.text('SIM joints differ from live MoveIt plans.')
        imgui.end()
