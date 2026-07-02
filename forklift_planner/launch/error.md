lsj@Lacia:~/lsj_ws/forklift_ws$ roslaunch forklift_planner multi_vehicle_batch.launch minutes:=120
... logging to /home/lsj/.ros/log/cd309fd0-665b-11f1-83c7-2149dec364f4/roslaunch-Lacia-9709.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://Lacia:33099/

SUMMARY
========

PARAMETERS
 * /forklift_map/arrow_size: 0.06
 * /forklift_map/bottom_shelf_depth: 0.25
 * /forklift_map/corridor_height: 0.6385
 * /forklift_map/field_height: 4.5
 * /forklift_map/field_width: 2.5
 * /forklift_map/frame_id: map
 * /forklift_map/max_steer_angle: 0.5
 * /forklift_map/max_steer_rate: 0.25
 * /forklift_map/path_resolution: 0.01
 * /forklift_map/pre_dock_clearance: 0.15
 * /forklift_map/road_line_width: 0.02
 * /forklift_map/row1_left_aisle: 0.58
 * /forklift_map/row1_mini_shelf: 0.2
 * /forklift_map/row1_shelf_width: 1.14
 * /forklift_map/row2_gap: 0.25
 * /forklift_map/row2_left_width: 1.125
 * /forklift_map/row3_center_aisle: 0.5
 * /forklift_map/row3_left_shelf: 1.0
 * /forklift_map/shelf_row_depth: 0.482
 * /forklift_map/tb_shelf_width: 1.125
 * /forklift_map/turn_speed: 0.2
 * /forklift_map/vehicle_length: 0.211
 * /forklift_map/vehicle_width: 0.191
 * /forklift_map/wheel_base: 0.143
 * /forklift_planner/frame_id: map
 * /forklift_planner/max_steer_angle: 0.5
 * /forklift_planner/max_steer_rate: 0.25
 * /forklift_planner/multi_vehicle/action_hold_time: 0.4
 * /forklift_planner/multi_vehicle/boost_ratio: 1.2
 * /forklift_planner/multi_vehicle/conflict_margin: 0.04
 * /forklift_planner/multi_vehicle/creep_ratio: 0.25
 * /forklift_planner/multi_vehicle/dwell_time: 10.0
 * /forklift_planner/multi_vehicle/emergency_time: 0.8
 * /forklift_planner/multi_vehicle/enable_boost: True
 * /forklift_planner/multi_vehicle/enable_cycle_break: False
 * /forklift_planner/multi_vehicle/enable_deadlock_recovery: True
 * /forklift_planner/multi_vehicle/enable_deadlock_reverse: False
 * /forklift_planner/multi_vehicle/enable_priority_tiebreak: True
 * /forklift_planner/multi_vehicle/enable_stall_release: False
 * /forklift_planner/multi_vehicle/final_decision_time: 2.0
 * /forklift_planner/multi_vehicle/following_creep_distance: 0.22
 * /forklift_planner/multi_vehicle/following_min_distance: 0.13
 * /forklift_planner/multi_vehicle/following_normal_distance: 0.35
 * /forklift_planner/multi_vehicle/kink_cusp_angle: 2.53
 * /forklift_planner/multi_vehicle/kink_min_angle: 0.61
 * /forklift_planner/multi_vehicle/log_invalid_task_pairs: False
 * /forklift_planner/multi_vehicle/max_accel: 0.2
 * /forklift_planner/multi_vehicle/max_decel: 0.3
 * /forklift_planner/multi_vehicle/max_speed: 0.26
 * /forklift_planner/multi_vehicle/nominal_speed: 0.2
 * /forklift_planner/multi_vehicle/path_validation_step: 0.02
 * /forklift_planner/multi_vehicle/precompute_task_filter: True
 * /forklift_planner/multi_vehicle/prediction_horizon: 10.0
 * /forklift_planner/multi_vehicle/prediction_step: 0.05
 * /forklift_planner/multi_vehicle/quiet_task_filter_precompute: True
 * /forklift_planner/multi_vehicle/random_seed: 2026
 * /forklift_planner/multi_vehicle/randomize_start: False
 * /forklift_planner/multi_vehicle/real_mode: False
 * /forklift_planner/multi_vehicle/recent_row_memory: 4
 * /forklift_planner/multi_vehicle/recent_target_memory: 5
 * /forklift_planner/multi_vehicle/reject_boundary_violations: True
 * /forklift_planner/multi_vehicle/reject_curvature_discontinuity: True
 * /forklift_planner/multi_vehicle/reject_path_kinks: True
 * /forklift_planner/multi_vehicle/reject_shelf_collisions: True
 * /forklift_planner/multi_vehicle/safety_margin: 0.0
 * /forklift_planner/multi_vehicle/show_paths: True
 * /forklift_planner/multi_vehicle/show_prediction_conflicts: True
 * /forklift_planner/multi_vehicle/simple_forward_demo: True
 * /forklift_planner/multi_vehicle/skip_arc_fallback_paths: True
 * /forklift_planner/multi_vehicle/start_slots: [38, 20, 36, 34, ...
 * /forklift_planner/multi_vehicle/starvation_wait_time: 8.0
 * /forklift_planner/multi_vehicle/target_request_distance: 0.45
 * /forklift_planner/multi_vehicle/target_slots: [9, 10, 8, 7, 40,...
 * /forklift_planner/multi_vehicle/target_stop_distance: 0.12
 * /forklift_planner/multi_vehicle/vehicle_count: 8
 * /forklift_planner/multi_vehicle/warning_time: 5.0
 * /forklift_planner/multi_vehicle/yield_ratio: 0.5
 * /forklift_planner/path_color_b: 1.0
 * /forklift_planner/path_color_g: 0.8
 * /forklift_planner/path_color_r: 0.2
 * /forklift_planner/path_resolution: 0.01
 * /forklift_planner/random_seed: 2026
 * /forklift_planner/stop_duration: 2.0
 * /forklift_planner/terminal_docking_mode: auto
 * /forklift_planner/turn_model: clothoid
 * /forklift_planner/update_rate: 10.0
 * /forklift_planner/vehicle_speed: 0.2
 * /forklift_planner/wheel_base: 0.143
 * /multi_vehicle_patrol_node/batch_minutes: 120
 * /rosdistro: noetic
 * /rosversion: 1.17.4

NODES
  /
    multi_vehicle_patrol_node (forklift_planner/multi_vehicle_patrol_node)

auto-starting new master
process[master]: started with pid [9717]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to cd309fd0-665b-11f1-83c7-2149dec364f4
process[rosout-1]: started with pid [9727]
started core service [/rosout]
process[multi_vehicle_patrol_node-2]: started with pid [9734]
[INFO] [1781267991.087672349]: [multi_patrol] task cache built: slots=66 pairs=4356 valid=933 same=66 empty=0 curvature=1907 boundary=121 shelf=1291 kink=38 valid_target_rows=[147,90,74,116,142,111,111,142]
[INFO] [1781267991.093609692]: [spin-check] 0 valid paths have mid-path heading jumps >52deg (0 jump points). worst=0deg at -->-
[WARN] [1781267991.120164189]: [simple-scan] 【走得近】 8 对(总长12.15m):
[WARN] [1781267991.120255194]: [simple-scan]   对1: 0 -> 10  len=1.227
[WARN] [1781267991.120287143]: [simple-scan]   对2: 17 -> 24  len=1.394
[WARN] [1781267991.120297035]: [simple-scan]   对3: 14 -> 7  len=1.399
[WARN] [1781267991.120321611]: [simple-scan]   对4: 31 -> 44  len=1.465
[WARN] [1781267991.120330680]: [simple-scan]   对5: 60 -> 51  len=1.484
[WARN] [1781267991.120336479]: [simple-scan]   对6: 46 -> 35  len=1.665
[WARN] [1781267991.120341841]: [simple-scan]   对7: 49 -> 58  len=1.682
[WARN] [1781267991.120379478]: [simple-scan]   对8: 12 -> 8  len=1.831
[WARN] [1781267991.120420161]: [simple-scan]   start_slots:  [0, 17, 14, 31, 60, 46, 49, 12]
[WARN] [1781267991.120460459]: [simple-scan]   target_slots: [10, 24, 7, 44, 51, 35, 58, 8]
[WARN] [1781267991.145154760]: [simple-scan] 【走得远】 8 对(总长35.62m):
[WARN] [1781267991.145242891]: [simple-scan]   对1: 38 -> 9  len=5.338
[WARN] [1781267991.145274272]: [simple-scan]   对2: 20 -> 10  len=5.156
[WARN] [1781267991.145283422]: [simple-scan]   对3: 36 -> 8  len=4.899
[WARN] [1781267991.145289475]: [simple-scan]   对4: 34 -> 7  len=4.461
[WARN] [1781267991.145295101]: [simple-scan]   对5: 56 -> 40  len=4.179
[WARN] [1781267991.145300402]: [simple-scan]   对6: 39 -> 55  len=4.160
[WARN] [1781267991.145305917]: [simple-scan]   对7: 57 -> 42  len=3.722
[WARN] [1781267991.145331113]: [simple-scan]   对8: 37 -> 53  len=3.702
[WARN] [1781267991.145370608]: [simple-scan]   start_slots:  [38, 20, 36, 34, 56, 39, 57, 37]
[WARN] [1781267991.145396840]: [simple-scan]   target_slots: [9, 10, 8, 7, 40, 55, 42, 53]
[INFO] [1781267991.145909232]: [multi_patrol] V0 task 0: slot 38 row=3 col=9 -> slot 9 row=0 col=9  wpts=308 len=5.338 arc=0
[INFO] [1781267991.145968916]: [multi_patrol][simple] V0 选最近前进目标 slot 9 row=0 col=9 len=5.338 (无尖点一把进)
[INFO] [1781267991.146060967]: [multi_patrol] V1 task 0: slot 20 row=3 col=0 -> slot 10 row=1 col=0  wpts=331 len=5.156 arc=0
[INFO] [1781267991.146114528]: [multi_patrol][simple] V1 选最近前进目标 slot 10 row=1 col=0 len=5.156 (无尖点一把进)
[INFO] [1781267991.146170282]: [multi_patrol] V2 task 0: slot 36 row=3 col=8 -> slot 8 row=0 col=8  wpts=308 len=4.899 arc=0
[INFO] [1781267991.146224828]: [multi_patrol][simple] V2 选最近前进目标 slot 8 row=0 col=8 len=4.899 (无尖点一把进)
[INFO] [1781267991.146308490]: [multi_patrol] V3 task 0: slot 34 row=3 col=7 -> slot 7 row=0 col=7  wpts=308 len=4.461 arc=0
[INFO] [1781267991.146362010]: [multi_patrol][simple] V3 选最近前进目标 slot 7 row=0 col=7 len=4.461 (无尖点一把进)
[INFO] [1781267991.146505377]: [multi_patrol] V4 task 0: slot 56 row=7 col=0 -> slot 40 row=5 col=0  wpts=333 len=4.179 arc=0
[INFO] [1781267991.146557364]: [multi_patrol][simple] V4 选最近前进目标 slot 40 row=5 col=0 len=4.179 (无尖点一把进)
[INFO] [1781267991.146641605]: [multi_patrol] V5 task 0: slot 39 row=4 col=9 -> slot 55 row=6 col=7  wpts=331 len=4.160 arc=0
[INFO] [1781267991.146693693]: [multi_patrol][simple] V5 选最近前进目标 slot 55 row=6 col=7 len=4.160 (无尖点一把进)
[INFO] [1781267991.146785185]: [multi_patrol] V6 task 0: slot 57 row=7 col=1 -> slot 42 row=5 col=1  wpts=333 len=3.722 arc=0
[INFO] [1781267991.146838014]: [multi_patrol][simple] V6 选最近前进目标 slot 42 row=5 col=1 len=3.722 (无尖点一把进)
[INFO] [1781267991.146922712]: [multi_patrol] V7 task 0: slot 37 row=4 col=8 -> slot 53 row=6 col=6  wpts=331 len=3.702 arc=0
[INFO] [1781267991.146978873]: [multi_patrol][simple] V7 选最近前进目标 slot 53 row=6 col=6 len=3.702 (无尖点一把进)
[INFO] [1781267991.147011178]: [res_map] resources built: 152
[INFO] [1781267991.147078662]: [res_map] V0 uses SLOT_BODY_9 s=[5.320,5.338] (len=5.338)
[INFO] [1781267991.147119244]: [res_map] V0 uses SLOT_DOCK_9 s=[5.040,5.338] (len=5.338)
[INFO] [1781267991.147128993]: [res_map] V0 uses SLOT_DOCK_10 s=[3.220,3.420] (len=5.338)
[INFO] [1781267991.147136366]: [res_map] V0 uses SLOT_DOCK_11 s=[2.120,2.300] (len=5.338)
[INFO] [1781267991.147142206]: [res_map] V0 uses SLOT_DOCK_12 s=[3.440,3.640] (len=5.338)
[INFO] [1781267991.147147944]: [res_map] V0 uses SLOT_DOCK_13 s=[1.880,2.080] (len=5.338)
[INFO] [1781267991.147185479]: [res_map] V0 uses SLOT_DOCK_14 s=[3.680,3.860] (len=5.338)
[INFO] [1781267991.147225390]: [res_map] V0 uses SLOT_DOCK_15 s=[1.660,1.860] (len=5.338)
[INFO] [1781267991.147236765]: [res_map] V0 uses SLOT_DOCK_16 s=[3.900,4.080] (len=5.338)
[INFO] [1781267991.147242421]: [res_map] V0 uses SLOT_DOCK_17 s=[1.440,1.640] (len=5.338)
[INFO] [1781267991.147280566]: [res_map] V0 uses SLOT_DOCK_18 s=[4.120,4.300] (len=5.338)
[INFO] [1781267991.147289919]: [res_map] V0 uses SLOT_DOCK_19 s=[1.220,1.420] (len=5.338)
[INFO] [1781267991.147296023]: [res_map] V0 uses SLOT_BODY_38 s=[0.000,0.180] (len=5.338)
[INFO] [1781267991.147333558]: [res_map] V0 uses SLOT_DOCK_38 s=[0.000,0.440] (len=5.338)
[INFO] [1781267991.147360216]: [res_map] V0 uses LANE_C2_lo s=[0.180,0.520] (len=5.338)
[INFO] [1781267991.147369722]: [res_map] V0 uses LANE_C2_hi s=[0.520,2.520] (len=5.338)
[INFO] [1781267991.147376486]: [res_map] V0 uses LANE_C1_lo s=[3.000,4.960] (len=5.338)
[INFO] [1781267991.147414996]: [res_map] V0 uses LANE_C1_hi s=[4.960,5.300] (len=5.338)
[INFO] [1781267991.147454786]: [res_map] V0 uses XING_C1_x1250 s=[3.680,4.060] (len=5.338)
[INFO] [1781267991.147465936]: [res_map] V0 uses XING_C2_x1250 s=[1.480,1.860] (len=5.338)
[INFO] [1781267991.147472385]: [res_map] V0 uses XING_C2_x290 s=[2.480,2.520] (len=5.338)
[INFO] [1781267991.147478641]: [res_map] V0 uses XING_C1_x290 s=[3.000,3.040] (len=5.338)
[INFO] [1781267991.147483953]: [res_map] V0 uses XING_C2_x2009 s=[0.700,1.100] (len=5.338)
[INFO] [1781267991.147521092]: [res_map] V0 uses XING_C1_x2009 s=[4.440,4.820] (len=5.338)
[INFO] [1781267991.147564223]: [res_map] V1 uses SLOT_DOCK_3 s=[4.660,4.700] (len=5.156)
[INFO] [1781267991.147601778]: [res_map] V1 uses SLOT_DOCK_4 s=[4.440,4.640] (len=5.156)
[INFO] [1781267991.147612157]: [res_map] V1 uses SLOT_DOCK_5 s=[3.940,4.140] (len=5.156)
[INFO] [1781267991.147617844]: [res_map] V1 uses SLOT_DOCK_6 s=[3.720,3.920] (len=5.156)
[INFO] [1781267991.147657207]: [res_map] V1 uses SLOT_DOCK_7 s=[3.520,3.700] (len=5.156)
[INFO] [1781267991.147696804]: [res_map] V1 uses SLOT_BODY_10 s=[5.140,5.156] (len=5.156)
[INFO] [1781267991.147707407]: [res_map] V1 uses SLOT_DOCK_10 s=[4.880,5.156] (len=5.156)
[INFO] [1781267991.147714902]: [res_map] V1 uses SLOT_DOCK_11 s=[1.020,1.220] (len=5.156)
[INFO] [1781267991.147752020]: [res_map] V1 uses SLOT_DOCK_13 s=[1.240,1.440] (len=5.156)
[INFO] [1781267991.147761364]: [res_map] V1 uses SLOT_DOCK_15 s=[1.460,1.660] (len=5.156)
[INFO] [1781267991.147784630]: [res_map] V1 uses SLOT_DOCK_17 s=[1.680,1.880] (len=5.156)
[INFO] [1781267991.147808374]: [res_map] V1 uses SLOT_DOCK_19 s=[1.920,2.100] (len=5.156)
[INFO] [1781267991.147834779]: [res_map] V1 uses SLOT_BODY_20 s=[0.000,0.180] (len=5.156)
[INFO] [1781267991.147882855]: [res_map] V1 uses SLOT_DOCK_20 s=[0.000,0.440] (len=5.156)
[INFO] [1781267991.147925468]: [res_map] V1 uses LANE_C2_lo s=[0.180,0.520] (len=5.156)
[INFO] [1781267991.147951365]: [res_map] V1 uses LANE_C2_hi s=[0.520,2.580] (len=5.156)
[INFO] [1781267991.147991246]: [res_map] V1 uses LANE_C1_lo s=[3.080,5.140] (len=5.156)
[INFO] [1781267991.148001564]: [res_map] V1 uses LANE_C1_hi s=[3.400,4.800] (len=5.156)
[INFO] [1781267991.148006632]: [res_map] V1 uses XING_C1_x1250 s=[4.100,4.480] (len=5.156)
[INFO] [1781267991.148043121]: [res_map] V1 uses XING_C2_x1250 s=[1.480,1.860] (len=5.156)
[INFO] [1781267991.148054110]: [res_map] V1 uses XING_C2_x290 s=[0.180,0.900] (len=5.156)
[INFO] [1781267991.148059522]: [res_map] V1 uses XING_C2_x2009 s=[2.240,2.580] (len=5.156)
[INFO] [1781267991.148064824]: [res_map] V1 uses XING_C1_x2009 s=[3.080,3.720] (len=5.156)
[INFO] [1781267991.148100541]: [res_map] V2 uses SLOT_BODY_8 s=[4.880,4.899] (len=4.899)
[INFO] [1781267991.148138198]: [res_map] V2 uses SLOT_DOCK_8 s=[4.600,4.899] (len=4.899)
[INFO] [1781267991.148148689]: [res_map] V2 uses SLOT_DOCK_10 s=[3.000,3.200] (len=4.899)
[INFO] [1781267991.148154478]: [res_map] V2 uses SLOT_DOCK_11 s=[1.900,2.080] (len=4.899)
[INFO] [1781267991.148179328]: [res_map] V2 uses SLOT_DOCK_12 s=[3.220,3.420] (len=4.899)
[INFO] [1781267991.148202869]: [res_map] V2 uses SLOT_DOCK_13 s=[1.660,1.860] (len=4.899)
[INFO] [1781267991.148212872]: [res_map] V2 uses SLOT_DOCK_14 s=[3.460,3.640] (len=4.899)
[INFO] [1781267991.148218935]: [res_map] V2 uses SLOT_DOCK_15 s=[1.440,1.640] (len=4.899)
[INFO] [1781267991.148225668]: [res_map] V2 uses SLOT_DOCK_16 s=[3.680,3.860] (len=4.899)
[INFO] [1781267991.148265235]: [res_map] V2 uses SLOT_DOCK_17 s=[1.220,1.420] (len=4.899)
[INFO] [1781267991.148291000]: [res_map] V2 uses SLOT_DOCK_18 s=[3.900,4.080] (len=4.899)
[INFO] [1781267991.148300779]: [res_map] V2 uses SLOT_DOCK_19 s=[1.000,1.200] (len=4.899)
[INFO] [1781267991.148337908]: [res_map] V2 uses SLOT_BODY_36 s=[0.000,0.180] (len=4.899)
[INFO] [1781267991.148377840]: [res_map] V2 uses SLOT_DOCK_36 s=[0.000,0.440] (len=4.899)
[INFO] [1781267991.148388514]: [res_map] V2 uses LANE_C2_lo s=[0.180,0.520] (len=4.899)
[INFO] [1781267991.148412024]: [res_map] V2 uses LANE_C2_hi s=[0.520,2.300] (len=4.899)
[INFO] [1781267991.148451631]: [res_map] V2 uses LANE_C1_lo s=[2.780,4.520] (len=4.899)
[INFO] [1781267991.148491431]: [res_map] V2 uses LANE_C1_hi s=[4.520,4.860] (len=4.899)
[INFO] [1781267991.148533780]: [res_map] V2 uses XING_C1_x1250 s=[3.460,3.840] (len=4.899)
[INFO] [1781267991.148559484]: [res_map] V2 uses XING_C2_x1250 s=[1.260,1.640] (len=4.899)
[INFO] [1781267991.148599070]: [res_map] V2 uses XING_C2_x290 s=[2.260,2.300] (len=4.899)
[INFO] [1781267991.148639510]: [res_map] V2 uses XING_C1_x290 s=[2.780,2.820] (len=4.899)
[INFO] [1781267991.148680488]: [res_map] V2 uses XING_C2_x2009 s=[0.180,0.880] (len=4.899)
[INFO] [1781267991.148690512]: [res_map] V2 uses XING_C1_x2009 s=[4.220,4.860] (len=4.899)
[INFO] [1781267991.148748734]: [res_map] V3 uses SLOT_BODY_7 s=[4.440,4.461] (len=4.461)
[INFO] [1781267991.148789346]: [res_map] V3 uses SLOT_DOCK_7 s=[4.180,4.461] (len=4.461)
[INFO] [1781267991.148800426]: [res_map] V3 uses SLOT_DOCK_10 s=[2.780,2.980] (len=4.461)
[INFO] [1781267991.148805524]: [res_map] V3 uses SLOT_DOCK_11 s=[1.680,1.860] (len=4.461)
[INFO] [1781267991.148811993]: [res_map] V3 uses SLOT_DOCK_12 s=[3.000,3.200] (len=4.461)
[INFO] [1781267991.148816695]: [res_map] V3 uses SLOT_DOCK_13 s=[1.440,1.640] (len=4.461)
[INFO] [1781267991.148821672]: [res_map] V3 uses SLOT_DOCK_14 s=[3.240,3.420] (len=4.461)
[INFO] [1781267991.148862121]: [res_map] V3 uses SLOT_DOCK_15 s=[1.220,1.420] (len=4.461)
[INFO] [1781267991.148903049]: [res_map] V3 uses SLOT_DOCK_16 s=[3.460,3.640] (len=4.461)
[INFO] [1781267991.148929890]: [res_map] V3 uses SLOT_DOCK_17 s=[1.000,1.200] (len=4.461)
[INFO] [1781267991.148955797]: [res_map] V3 uses SLOT_DOCK_18 s=[3.680,3.860] (len=4.461)
[INFO] [1781267991.149001264]: [res_map] V3 uses SLOT_DOCK_19 s=[0.780,0.980] (len=4.461)
[INFO] [1781267991.149043166]: [res_map] V3 uses SLOT_BODY_34 s=[0.000,0.180] (len=4.461)
[INFO] [1781267991.149053464]: [res_map] V3 uses SLOT_DOCK_34 s=[0.000,0.440] (len=4.461)
[INFO] [1781267991.149059262]: [res_map] V3 uses LANE_C2_lo s=[0.180,0.520] (len=4.461)
[INFO] [1781267991.149083545]: [res_map] V3 uses LANE_C2_hi s=[0.520,2.080] (len=4.461)
[INFO] [1781267991.149124157]: [res_map] V3 uses LANE_C1_lo s=[2.560,4.080] (len=4.461)
[INFO] [1781267991.149165856]: [res_map] V3 uses LANE_C1_hi s=[4.080,4.420] (len=4.461)
[INFO] [1781267991.149175890]: [res_map] V3 uses XING_C1_x1250 s=[3.240,3.620] (len=4.461)
[INFO] [1781267991.149200466]: [res_map] V3 uses XING_C2_x1250 s=[1.040,1.420] (len=4.461)
[INFO] [1781267991.149241282]: [res_map] V3 uses XING_C2_x290 s=[2.040,2.080] (len=4.461)
[INFO] [1781267991.149282361]: [res_map] V3 uses XING_C1_x290 s=[2.560,2.600] (len=4.461)
[INFO] [1781267991.149292669]: [res_map] V3 uses XING_C2_x2009 s=[0.180,0.640] (len=4.461)
[INFO] [1781267991.149298326]: [res_map] V3 uses XING_C1_x2009 s=[4.020,4.420] (len=4.461)
[INFO] [1781267991.149331545]: [res_map] V4 uses SLOT_DOCK_23 s=[3.560,3.720] (len=4.179)
[INFO] [1781267991.149370563]: [res_map] V4 uses SLOT_DOCK_25 s=[3.340,3.520] (len=4.179)
[INFO] [1781267991.149397445]: [res_map] V4 uses SLOT_DOCK_27 s=[3.120,3.300] (len=4.179)
[INFO] [1781267991.149407885]: [res_map] V4 uses SLOT_DOCK_29 s=[2.900,3.080] (len=4.179)
[INFO] [1781267991.149446212]: [res_map] V4 uses SLOT_BODY_40 s=[4.160,4.179] (len=4.179)
[INFO] [1781267991.149487779]: [res_map] V4 uses SLOT_DOCK_40 s=[3.900,4.179] (len=4.179)
[INFO] [1781267991.149514549]: [res_map] V4 uses SLOT_DOCK_41 s=[0.620,0.660] (len=4.179)
[INFO] [1781267991.149556766]: [res_map] V4 uses SLOT_DOCK_43 s=[0.720,0.900] (len=4.179)
[INFO] [1781267991.149585425]: [res_map] V4 uses SLOT_DOCK_45 s=[0.960,1.140] (len=4.179)
[INFO] [1781267991.149594220]: [res_map] V4 uses SLOT_DOCK_47 s=[1.200,1.380] (len=4.179)
[INFO] [1781267991.149633583]: [res_map] V4 uses SLOT_BODY_56 s=[0.000,0.180] (len=4.179)
[INFO] [1781267991.149677364]: [res_map] V4 uses SLOT_DOCK_56 s=[0.000,0.460] (len=4.179)
[INFO] [1781267991.149707415]: [res_map] V4 uses LANE_C4_lo s=[0.200,0.540] (len=4.179)
[INFO] [1781267991.149748921]: [res_map] V4 uses LANE_C4_hi s=[0.540,1.820] (len=4.179)
[INFO] [1781267991.149790092]: [res_map] V4 uses LANE_C3_lo s=[2.320,4.160] (len=4.179)
[INFO] [1781267991.149816638]: [res_map] V4 uses LANE_C3_hi s=[2.640,3.820] (len=4.179)
[INFO] [1781267991.149857789]: [res_map] V4 uses XING_C3_x1250 s=[2.320,2.940] (len=4.179)
[INFO] [1781267991.149901763]: [res_map] V4 uses XING_C4_x1250 s=[1.500,1.820] (len=4.179)
[INFO] [1781267991.149928432]: [res_map] V4 uses XING_C3_x1250 s=[2.320,2.940] (len=4.179)
[INFO] [1781267991.149966901]: [res_map] V5 uses SLOT_BODY_39 s=[0.000,0.180] (len=4.160)
[INFO] [1781267991.150006112]: [res_map] V5 uses SLOT_DOCK_39 s=[0.000,0.440] (len=4.160)
[INFO] [1781267991.150017699]: [res_map] V5 uses SLOT_DOCK_48 s=[1.180,1.360] (len=4.160)
[INFO] [1781267991.150024798]: [res_map] V5 uses SLOT_DOCK_50 s=[0.940,1.120] (len=4.160)
[INFO] [1781267991.150029399]: [res_map] V5 uses SLOT_DOCK_52 s=[0.700,0.880] (len=4.160)
[INFO] [1781267991.150068031]: [res_map] V5 uses SLOT_DOCK_54 s=[0.600,0.640] (len=4.160)
[INFO] [1781267991.150108856]: [res_map] V5 uses SLOT_BODY_55 s=[4.140,4.160] (len=4.160)
[INFO] [1781267991.150136754]: [res_map] V5 uses SLOT_DOCK_55 s=[3.880,4.160] (len=4.160)
[INFO] [1781267991.150177427]: [res_map] V5 uses SLOT_DOCK_61 s=[2.880,3.060] (len=4.160)
[INFO] [1781267991.150203913]: [res_map] V5 uses SLOT_DOCK_62 s=[3.100,3.280] (len=4.160)
[INFO] [1781267991.150245348]: [res_map] V5 uses SLOT_DOCK_63 s=[3.320,3.500] (len=4.160)
[INFO] [1781267991.150288377]: [res_map] V5 uses SLOT_DOCK_64 s=[3.540,3.700] (len=4.160)
[INFO] [1781267991.150315737]: [res_map] V5 uses LANE_C4_lo s=[2.620,3.800] (len=4.160)
[INFO] [1781267991.150356999]: [res_map] V5 uses LANE_C4_hi s=[2.300,4.140] (len=4.160)
[INFO] [1781267991.150367520]: [res_map] V5 uses LANE_C3_lo s=[0.520,1.800] (len=4.160)
[INFO] [1781267991.150407594]: [res_map] V5 uses LANE_C3_hi s=[0.180,0.520] (len=4.160)
[INFO] [1781267991.150436131]: [res_map] V5 uses XING_C3_x1250 s=[1.480,1.800] (len=4.160)
[INFO] [1781267991.150461124]: [res_map] V5 uses XING_C4_x1250 s=[2.300,2.920] (len=4.160)
[INFO] [1781267991.150501696]: [res_map] V5 uses XING_C3_x1250 s=[1.480,1.800] (len=4.160)
[INFO] [1781267991.150555683]: [res_map] V6 uses SLOT_DOCK_25 s=[3.120,3.260] (len=3.722)
[INFO] [1781267991.150596905]: [res_map] V6 uses SLOT_DOCK_27 s=[2.900,3.080] (len=3.722)
[INFO] [1781267991.150623513]: [res_map] V6 uses SLOT_DOCK_29 s=[2.680,2.860] (len=3.722)
[INFO] [1781267991.150632755]: [res_map] V6 uses SLOT_BODY_42 s=[3.700,3.722] (len=3.722)
[INFO] [1781267991.150671387]: [res_map] V6 uses SLOT_DOCK_42 s=[3.460,3.722] (len=3.722)
[INFO] [1781267991.150682172]: [res_map] V6 uses SLOT_DOCK_43 s=[0.620,0.680] (len=3.722)
[INFO] [1781267991.150721322]: [res_map] V6 uses SLOT_DOCK_45 s=[0.740,0.920] (len=3.722)
[INFO] [1781267991.150732046]: [res_map] V6 uses SLOT_DOCK_47 s=[0.980,1.160] (len=3.722)
[INFO] [1781267991.150738089]: [res_map] V6 uses SLOT_BODY_57 s=[0.000,0.180] (len=3.722)
[INFO] [1781267991.150743278]: [res_map] V6 uses SLOT_DOCK_57 s=[0.000,0.460] (len=3.722)
[INFO] [1781267991.150748569]: [res_map] V6 uses LANE_C4_lo s=[0.200,0.540] (len=3.722)
[INFO] [1781267991.150754338]: [res_map] V6 uses LANE_C4_hi s=[0.540,1.600] (len=3.722)
[INFO] [1781267991.150797662]: [res_map] V6 uses LANE_C3_lo s=[2.100,3.700] (len=3.722)
[INFO] [1781267991.150808254]: [res_map] V6 uses LANE_C3_hi s=[2.420,3.360] (len=3.722)
[INFO] [1781267991.150814073]: [res_map] V6 uses XING_C3_x1250 s=[2.100,2.720] (len=3.722)
[INFO] [1781267991.150838315]: [res_map] V6 uses XING_C4_x1250 s=[1.280,1.600] (len=3.722)
[INFO] [1781267991.150878439]: [res_map] V6 uses XING_C3_x1250 s=[2.100,2.720] (len=3.722)
[INFO] [1781267991.150930060]: [res_map] V7 uses SLOT_BODY_37 s=[0.000,0.180] (len=3.702)
[INFO] [1781267991.150970886]: [res_map] V7 uses SLOT_DOCK_37 s=[0.000,0.440] (len=3.702)
[INFO] [1781267991.150997423]: [res_map] V7 uses SLOT_DOCK_48 s=[0.960,1.140] (len=3.702)
[INFO] [1781267991.151007903]: [res_map] V7 uses SLOT_DOCK_50 s=[0.720,0.900] (len=3.702)
[INFO] [1781267991.151050080]: [res_map] V7 uses SLOT_DOCK_52 s=[0.600,0.660] (len=3.702)
[INFO] [1781267991.151076850]: [res_map] V7 uses SLOT_BODY_53 s=[3.680,3.702] (len=3.702)
[INFO] [1781267991.151118011]: [res_map] V7 uses SLOT_DOCK_53 s=[3.440,3.702] (len=3.702)
[INFO] [1781267991.151146964]: [res_map] V7 uses SLOT_DOCK_61 s=[2.660,2.840] (len=3.702)
[INFO] [1781267991.151155353]: [res_map] V7 uses SLOT_DOCK_62 s=[2.880,3.060] (len=3.702)
[INFO] [1781267991.151194107]: [res_map] V7 uses SLOT_DOCK_63 s=[3.100,3.240] (len=3.702)
[INFO] [1781267991.151220379]: [res_map] V7 uses LANE_C4_lo s=[2.400,3.340] (len=3.702)
[INFO] [1781267991.151228544]: [res_map] V7 uses LANE_C4_hi s=[2.080,3.680] (len=3.702)
[INFO] [1781267991.151267654]: [res_map] V7 uses LANE_C3_lo s=[0.520,1.580] (len=3.702)
[INFO] [1781267991.151294160]: [res_map] V7 uses LANE_C3_hi s=[0.180,0.520] (len=3.702)
[INFO] [1781267991.151325510]: [res_map] V7 uses XING_C3_x1250 s=[1.260,1.580] (len=3.702)
[INFO] [1781267991.151333645]: [res_map] V7 uses XING_C4_x1250 s=[2.080,2.700] (len=3.702)
[INFO] [1781267991.151340520]: [res_map] V7 uses XING_C3_x1250 s=[1.260,1.580] (len=3.702)
[WARN] [1781267991.152175221]: [batch] 无头快速回归模式:将狂跑 72000 拍(≈120 仿真分钟),不发 marker、不按实时。
[WARN] [1781267991.204278889]: [DIAG wedge] V0(s=1.443,rem=3.895,wait=0.0,act=3) vs V2(s=0.007,rem=4.892,wait=8.1,act=0) owner=V0 nzones=1 | A region[se=0.000 sx=5.325] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.204331181]: [DIAG wedge]   zone0 A[0.000,5.325] B[0.000,4.875] @(2.27,4.26)
[WARN] [1781267991.204361160]: [DIAG wedge] V0(s=1.443,rem=3.895,wait=0.0,act=3) vs V3(s=0.000,rem=4.461,wait=8.1,act=0) owner=V0 nzones=1 | A region[se=0.650 sx=4.725] stopline=0.471 committed=1 | B region[se=0.325 sx=4.150] stopline=0.146 committed=0 | front_ext=0.179
[WARN] [1781267991.204402534]: [DIAG wedge]   zone0 A[0.650,4.725] B[0.325,4.150] @(1.96,3.85)
[WARN] [1781267991.204414538]: [DIAG wedge] V1(s=0.396,rem=4.760,wait=6.1,act=0) vs V2(s=0.007,rem=4.892,wait=8.1,act=0) owner=V1 nzones=3 | A region[se=0.575 sx=5.150] stopline=0.396 committed=0 | B region[se=0.375 sx=4.800] stopline=0.196 committed=0 | front_ext=0.179
[WARN] [1781267991.204441268]: [DIAG wedge]   zone0 A[0.575,2.550] B[0.375,2.325] @(2.06,2.99)
[WARN] [1781267991.204482164]: [DIAG wedge]   zone1 A[2.975,3.625] B[4.150,4.800] @(1.98,3.98)
[WARN] [1781267991.204508965]: [DIAG wedge]   zone2 A[4.500,5.150] B[2.650,3.325] @(0.69,3.69)
[WARN] [1781267991.204521690]: [DIAG wedge] V1(s=0.396,rem=4.760,wait=6.1,act=0) vs V3(s=0.000,rem=4.461,wait=8.1,act=0) owner=V1 nzones=3 | A region[se=0.575 sx=5.150] stopline=0.396 committed=0 | B region[se=0.325 sx=4.425] stopline=0.146 committed=0 | front_ext=0.179
[WARN] [1781267991.204527875]: [DIAG wedge]   zone0 A[0.575,2.425] B[0.325,2.125] @(1.94,2.91)
[WARN] [1781267991.204534882]: [DIAG wedge]   zone1 A[3.175,3.825] B[3.825,4.425] @(1.80,4.02)
[WARN] [1781267991.204539706]: [DIAG wedge]   zone2 A[4.500,5.150] B[2.450,3.125] @(0.69,3.69)
[WARN] [1781267991.204569818]: [DIAG wedge] V2(s=0.007,rem=4.892,wait=8.1,act=0) vs V3(s=0.000,rem=4.461,wait=8.1,act=0) owner=V2 nzones=1 | A region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.450] stopline=-0.179 committed=0 | front_ext=0.179
[WARN] [1781267991.204598659]: [DIAG wedge]   zone0 A[0.000,4.875] B[0.000,4.450] @(2.05,4.26)
[WARN] [1781267991.204610105]: [DIAG wedge] V4(s=1.443,rem=2.736,wait=0.0,act=3) vs V5(s=0.000,rem=4.160,wait=8.1,act=0) owner=V4 nzones=2 | A region[se=1.250 sx=2.700] stopline=1.071 committed=1 | B region[se=1.225 sx=2.675] stopline=1.046 committed=0 | front_ext=0.179
[WARN] [1781267991.204655876]: [DIAG wedge]   zone0 A[1.250,1.925] B[2.025,2.675] @(1.24,0.95)
[WARN] [1781267991.204683550]: [DIAG wedge]   zone1 A[2.050,2.700] B[1.225,1.900] @(1.33,1.62)
[WARN] [1781267991.204706746]: [DIAG wedge] V4(s=1.443,rem=2.736,wait=0.0,act=3) vs V6(s=0.013,rem=3.709,wait=8.1,act=0) owner=V4 nzones=1 | A region[se=0.000 sx=4.175] stopline=-0.179 committed=1 | B region[se=0.000 sx=3.600] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.204749217]: [DIAG wedge]   zone0 A[0.000,4.175] B[0.000,3.600] @(0.26,1.43)
[WARN] [1781267991.204778983]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=8.1,act=0) vs V6(s=0.013,rem=3.709,wait=8.1,act=0) owner=V5 nzones=2 | A region[se=1.225 sx=2.675] stopline=1.046 committed=0 | B region[se=1.025 sx=2.475] stopline=0.846 committed=0 | front_ext=0.179
[WARN] [1781267991.204821423]: [DIAG wedge]   zone0 A[1.225,1.900] B[1.825,2.475] @(1.26,1.32)
[WARN] [1781267991.204862960]: [DIAG wedge]   zone1 A[2.025,2.675] B[1.025,1.700] @(1.17,0.64)
[WARN] [1781267991.204892238]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=8.1,act=0) vs V7(s=0.822,rem=2.880,wait=3.4,act=0) owner=V7 nzones=1 | A region[se=0.000 sx=4.150] stopline=-0.179 committed=0 | B region[se=0.000 sx=3.575] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.204934597]: [DIAG wedge]   zone0 A[0.000,4.150] B[0.000,3.575] @(2.24,0.83)
[WARN] [1781267991.204964424]: [DIAG wedge] V6(s=0.013,rem=3.709,wait=8.1,act=0) vs V7(s=0.822,rem=2.880,wait=3.4,act=0) owner=V7 nzones=2 | A region[se=1.025 sx=2.475] stopline=0.846 committed=0 | B region[se=1.000 sx=2.475] stopline=0.821 committed=0 | front_ext=0.179
[WARN] [1781267991.205007241]: [DIAG wedge]   zone0 A[1.025,1.700] B[1.800,2.475] @(1.24,0.95)
[WARN] [1781267991.205034143]: [DIAG wedge]   zone1 A[1.825,2.475] B[1.000,1.675] @(1.33,1.62)
[WARN] [1781267991.205584496]: [DIAG wedge] V0(s=1.843,rem=3.495,wait=0.0,act=3) vs V1(s=0.396,rem=4.760,wait=8.1,act=0) owner=V0 nzones=3 | A region[se=0.525 sx=5.025] stopline=0.346 committed=1 | B region[se=0.575 sx=5.150] stopline=0.396 committed=0 | front_ext=0.179
[WARN] [1781267991.205631080]: [DIAG wedge]   zone0 A[0.525,2.550] B[0.575,2.625] @(0.41,3.04)
[WARN] [1781267991.205642668]: [DIAG wedge]   zone1 A[2.875,3.550] B[4.500,5.150] @(0.84,3.84)
[WARN] [1781267991.205647928]: [DIAG wedge]   zone2 A[4.375,5.025] B[2.950,3.550] @(2.23,3.90)
[WARN] [1781267991.205955654]: [DIAG wedge] V0(s=2.063,rem=3.275,wait=0.0,act=3) vs V2(s=0.007,rem=4.892,wait=11.2,act=0) owner=V0 nzones=1 | A region[se=0.000 sx=5.325] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.205996144]: [DIAG wedge]   zone0 A[0.000,5.325] B[0.000,4.875] @(2.27,4.26)
[WARN] [1781267991.206026865]: [DIAG wedge] V0(s=2.063,rem=3.275,wait=0.0,act=3) vs V3(s=0.000,rem=4.461,wait=11.2,act=0) owner=V0 nzones=1 | A region[se=0.650 sx=4.725] stopline=0.471 committed=1 | B region[se=0.325 sx=4.150] stopline=0.146 committed=0 | front_ext=0.179
[WARN] [1781267991.206068371]: [DIAG wedge]   zone0 A[0.650,4.725] B[0.325,4.150] @(1.96,3.85)
[WARN] [1781267991.206112761]: [DIAG wedge] V1(s=0.396,rem=4.760,wait=9.2,act=0) vs V2(s=0.007,rem=4.892,wait=11.2,act=0) owner=V1 nzones=3 | A region[se=0.575 sx=5.150] stopline=0.396 committed=0 | B region[se=0.375 sx=4.800] stopline=0.196 committed=0 | front_ext=0.179
[WARN] [1781267991.206143259]: [DIAG wedge]   zone0 A[0.575,2.550] B[0.375,2.325] @(2.06,2.99)
[WARN] [1781267991.206167297]: [DIAG wedge]   zone1 A[2.975,3.625] B[4.150,4.800] @(1.98,3.98)
[WARN] [1781267991.206193732]: [DIAG wedge]   zone2 A[4.500,5.150] B[2.650,3.325] @(0.69,3.69)
[WARN] [1781267991.206236762]: [DIAG wedge] V1(s=0.396,rem=4.760,wait=9.2,act=0) vs V3(s=0.000,rem=4.461,wait=11.2,act=0) owner=V1 nzones=3 | A region[se=0.575 sx=5.150] stopline=0.396 committed=0 | B region[se=0.325 sx=4.425] stopline=0.146 committed=0 | front_ext=0.179
[WARN] [1781267991.206264639]: [DIAG wedge]   zone0 A[0.575,2.425] B[0.325,2.125] @(1.94,2.91)
[WARN] [1781267991.206308704]: [DIAG wedge]   zone1 A[3.175,3.825] B[3.825,4.425] @(1.80,4.02)
[WARN] [1781267991.206353643]: [DIAG wedge]   zone2 A[4.500,5.150] B[2.450,3.125] @(0.69,3.69)
[WARN] [1781267991.206383957]: [DIAG wedge] V2(s=0.007,rem=4.892,wait=11.2,act=0) vs V3(s=0.000,rem=4.461,wait=11.2,act=0) owner=V2 nzones=1 | A region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.450] stopline=-0.179 committed=0 | front_ext=0.179
[WARN] [1781267991.206411509]: [DIAG wedge]   zone0 A[0.000,4.875] B[0.000,4.450] @(2.05,4.26)
[WARN] [1781267991.206457586]: [DIAG wedge] V4(s=2.027,rem=2.152,wait=0.0,act=3) vs V5(s=0.000,rem=4.160,wait=11.2,act=0) owner=V4 nzones=1 | A region[se=2.050 sx=2.700] stopline=1.871 committed=0 | B region[se=1.225 sx=1.900] stopline=1.046 committed=0 | front_ext=0.179
[WARN] [1781267991.206468899]: [DIAG wedge]   zone0 A[2.050,2.700] B[1.225,1.900] @(1.33,1.62)
[WARN] [1781267991.206510354]: [DIAG wedge] V4(s=2.027,rem=2.152,wait=0.0,act=3) vs V6(s=0.013,rem=3.709,wait=11.2,act=0) owner=V4 nzones=1 | A region[se=0.000 sx=4.175] stopline=-0.179 committed=1 | B region[se=0.000 sx=3.600] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.206538424]: [DIAG wedge]   zone0 A[0.000,4.175] B[0.000,3.600] @(0.26,1.43)
[WARN] [1781267991.206550428]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=11.2,act=0) vs V6(s=0.013,rem=3.709,wait=11.2,act=0) owner=V5 nzones=2 | A region[se=1.225 sx=2.675] stopline=1.046 committed=0 | B region[se=1.025 sx=2.475] stopline=0.846 committed=0 | front_ext=0.179
[WARN] [1781267991.206592584]: [DIAG wedge]   zone0 A[1.225,1.900] B[1.825,2.475] @(1.26,1.32)
[WARN] [1781267991.206634365]: [DIAG wedge]   zone1 A[2.025,2.675] B[1.025,1.700] @(1.17,0.64)
[WARN] [1781267991.206665756]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=11.2,act=0) vs V7(s=0.822,rem=2.880,wait=6.5,act=0) owner=V7 nzones=1 | A region[se=0.000 sx=4.150] stopline=-0.179 committed=0 | B region[se=0.000 sx=3.575] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.206692902]: [DIAG wedge]   zone0 A[0.000,4.150] B[0.000,3.575] @(2.24,0.83)
[WARN] [1781267991.206705535]: [DIAG wedge] V6(s=0.013,rem=3.709,wait=11.2,act=0) vs V7(s=0.822,rem=2.880,wait=6.5,act=0) owner=V7 nzones=2 | A region[se=1.025 sx=2.475] stopline=0.846 committed=0 | B region[se=1.000 sx=2.475] stopline=0.821 committed=0 | front_ext=0.179
[WARN] [1781267991.206754455]: [DIAG wedge]   zone0 A[1.025,1.700] B[1.800,2.475] @(1.24,0.95)
[WARN] [1781267991.206767271]: [DIAG wedge]   zone1 A[1.825,2.475] B[1.000,1.675] @(1.33,1.62)
[WARN] [1781267991.207190679]: [DIAG wedge] V4(s=2.347,rem=1.832,wait=0.0,act=3) vs V7(s=0.822,rem=2.880,wait=8.1,act=0) owner=V4 nzones=1 | A region[se=2.050 sx=2.700] stopline=1.871 committed=1 | B region[se=1.000 sx=1.675] stopline=0.821 committed=0 | front_ext=0.179
[WARN] [1781267991.207232348]: [DIAG wedge]   zone0 A[2.050,2.700] B[1.000,1.675] @(1.33,1.62)
[WARN] [1781267991.207374202]: [DIAG wedge] V0(s=2.431,rem=2.907,wait=0.0,act=3) vs V1(s=0.396,rem=4.760,wait=11.2,act=0) owner=V0 nzones=3 | A region[se=0.525 sx=5.025] stopline=0.346 committed=1 | B region[se=0.575 sx=5.150] stopline=0.396 committed=0 | front_ext=0.179
[WARN] [1781267991.207416175]: [DIAG wedge]   zone0 A[0.525,2.550] B[0.575,2.625] @(0.41,3.04)
[WARN] [1781267991.207442955]: [DIAG wedge]   zone1 A[2.875,3.550] B[4.500,5.150] @(0.84,3.84)
[WARN] [1781267991.207451913]: [DIAG wedge]   zone2 A[4.375,5.025] B[2.950,3.550] @(2.23,3.90)
[WARN] [1781267991.207807410]: [DIAG wedge] V0(s=2.628,rem=2.710,wait=0.0,act=3) vs V2(s=0.007,rem=4.892,wait=14.3,act=0) owner=V0 nzones=1 | A region[se=0.000 sx=5.325] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.207851750]: [DIAG wedge]   zone0 A[0.000,5.325] B[0.000,4.875] @(2.27,4.26)
[WARN] [1781267991.207917507]: [DIAG wedge] V0(s=2.628,rem=2.710,wait=0.0,act=3) vs V3(s=0.000,rem=4.461,wait=14.3,act=0) owner=V0 nzones=1 | A region[se=0.650 sx=4.725] stopline=0.471 committed=1 | B region[se=0.325 sx=4.150] stopline=0.146 committed=0 | front_ext=0.179
[WARN] [1781267991.207959978]: [DIAG wedge]   zone0 A[0.650,4.725] B[0.325,4.150] @(1.96,3.85)
[WARN] [1781267991.207975841]: [DIAG wedge] V1(s=0.402,rem=4.754,wait=12.3,act=1) vs V2(s=0.007,rem=4.892,wait=14.3,act=0) owner=V1 nzones=3 | A region[se=0.575 sx=5.150] stopline=0.396 committed=0 | B region[se=0.375 sx=4.800] stopline=0.196 committed=0 | front_ext=0.179
[WARN] [1781267991.208022780]: [DIAG wedge]   zone0 A[0.575,2.550] B[0.375,2.325] @(2.06,2.99)
[WARN] [1781267991.208066754]: [DIAG wedge]   zone1 A[2.975,3.625] B[4.150,4.800] @(1.98,3.98)
[WARN] [1781267991.208096348]: [DIAG wedge]   zone2 A[4.500,5.150] B[2.650,3.325] @(0.69,3.69)
[WARN] [1781267991.208123656]: [DIAG wedge] V1(s=0.402,rem=4.754,wait=12.3,act=1) vs V3(s=0.000,rem=4.461,wait=14.3,act=0) owner=V1 nzones=3 | A region[se=0.575 sx=5.150] stopline=0.396 committed=0 | B region[se=0.325 sx=4.425] stopline=0.146 committed=0 | front_ext=0.179
[WARN] [1781267991.208167356]: [DIAG wedge]   zone0 A[0.575,2.425] B[0.325,2.125] @(1.94,2.91)
[WARN] [1781267991.208198290]: [DIAG wedge]   zone1 A[3.175,3.825] B[3.825,4.425] @(1.80,4.02)
[WARN] [1781267991.208241258]: [DIAG wedge]   zone2 A[4.500,5.150] B[2.450,3.125] @(0.69,3.69)
[WARN] [1781267991.208288888]: [DIAG wedge] V2(s=0.007,rem=4.892,wait=14.3,act=0) vs V3(s=0.000,rem=4.461,wait=14.3,act=0) owner=V2 nzones=1 | A region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.450] stopline=-0.179 committed=0 | front_ext=0.179
[WARN] [1781267991.208299501]: [DIAG wedge]   zone0 A[0.000,4.875] B[0.000,4.450] @(2.05,4.26)
[WARN] [1781267991.208327398]: [DIAG wedge] V4(s=2.640,rem=1.539,wait=0.0,act=3) vs V5(s=0.000,rem=4.160,wait=14.3,act=0) owner=V4 nzones=1 | A region[se=2.050 sx=2.700] stopline=1.871 committed=1 | B region[se=1.225 sx=1.900] stopline=1.046 committed=0 | front_ext=0.179
[WARN] [1781267991.208371047]: [DIAG wedge]   zone0 A[2.050,2.700] B[1.225,1.900] @(1.33,1.62)
[WARN] [1781267991.208414696]: [DIAG wedge] V4(s=2.640,rem=1.539,wait=0.0,act=3) vs V6(s=0.013,rem=3.709,wait=14.3,act=0) owner=V4 nzones=1 | A region[se=0.000 sx=4.175] stopline=-0.179 committed=1 | B region[se=0.000 sx=3.600] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.208425431]: [DIAG wedge]   zone0 A[0.000,4.175] B[0.000,3.600] @(0.26,1.43)
[WARN] [1781267991.208455847]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=14.3,act=0) vs V6(s=0.013,rem=3.709,wait=14.3,act=0) owner=V5 nzones=2 | A region[se=1.225 sx=2.675] stopline=1.046 committed=0 | B region[se=1.025 sx=2.475] stopline=0.846 committed=0 | front_ext=0.179
[WARN] [1781267991.208496611]: [DIAG wedge]   zone0 A[1.225,1.900] B[1.825,2.475] @(1.26,1.32)
[WARN] [1781267991.208540108]: [DIAG wedge]   zone1 A[2.025,2.675] B[1.025,1.700] @(1.17,0.64)
[WARN] [1781267991.208569387]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=14.3,act=0) vs V7(s=0.822,rem=2.880,wait=9.6,act=0) owner=V7 nzones=1 | A region[se=0.000 sx=4.150] stopline=-0.179 committed=0 | B region[se=0.000 sx=3.575] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.208610740]: [DIAG wedge]   zone0 A[0.000,4.150] B[0.000,3.575] @(2.24,0.83)
[WARN] [1781267991.208660147]: [DIAG wedge] V6(s=0.013,rem=3.709,wait=14.3,act=0) vs V7(s=0.822,rem=2.880,wait=9.6,act=0) owner=V7 nzones=2 | A region[se=1.025 sx=2.475] stopline=0.846 committed=0 | B region[se=1.000 sx=2.475] stopline=0.821 committed=0 | front_ext=0.179
[WARN] [1781267991.208689040]: [DIAG wedge]   zone0 A[1.025,1.700] B[1.800,2.475] @(1.24,0.95)
[WARN] [1781267991.208731602]: [DIAG wedge]   zone1 A[1.825,2.475] B[1.000,1.675] @(1.33,1.62)
[WARN] [1781267991.210024769]: [DIAG wedge] V0(s=3.185,rem=2.153,wait=0.0,act=3) vs V2(s=0.007,rem=4.892,wait=17.3,act=0) owner=V0 nzones=1 | A region[se=0.000 sx=5.325] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.210067950]: [DIAG wedge]   zone0 A[0.000,5.325] B[0.000,4.875] @(2.27,4.26)
[WARN] [1781267991.210098011]: [DIAG wedge] V0(s=3.185,rem=2.153,wait=0.0,act=3) vs V3(s=0.000,rem=4.461,wait=17.3,act=0) owner=V0 nzones=1 | A region[se=0.650 sx=4.725] stopline=0.471 committed=1 | B region[se=0.325 sx=4.150] stopline=0.146 committed=0 | front_ext=0.179
[WARN] [1781267991.210141406]: [DIAG wedge]   zone0 A[0.650,4.725] B[0.325,4.150] @(1.96,3.85)
[WARN] [1781267991.210171639]: [DIAG wedge] V1(s=0.840,rem=4.316,wait=0.0,act=3) vs V2(s=0.007,rem=4.892,wait=17.3,act=0) owner=V1 nzones=3 | A region[se=0.575 sx=5.150] stopline=0.396 committed=1 | B region[se=0.375 sx=4.800] stopline=0.196 committed=0 | front_ext=0.179
[WARN] [1781267991.210214709]: [DIAG wedge]   zone0 A[0.575,2.550] B[0.375,2.325] @(2.06,2.99)
[WARN] [1781267991.210244516]: [DIAG wedge]   zone1 A[2.975,3.625] B[4.150,4.800] @(1.98,3.98)
[WARN] [1781267991.210253920]: [DIAG wedge]   zone2 A[4.500,5.150] B[2.650,3.325] @(0.69,3.69)
[WARN] [1781267991.210296157]: [DIAG wedge] V1(s=0.840,rem=4.316,wait=0.0,act=3) vs V3(s=0.000,rem=4.461,wait=17.3,act=0) owner=V1 nzones=3 | A region[se=0.575 sx=5.150] stopline=0.396 committed=1 | B region[se=0.325 sx=4.425] stopline=0.146 committed=0 | front_ext=0.179
[WARN] [1781267991.210338628]: [DIAG wedge]   zone0 A[0.575,2.425] B[0.325,2.125] @(1.94,2.91)
[WARN] [1781267991.210350449]: [DIAG wedge]   zone1 A[3.175,3.825] B[3.825,4.425] @(1.80,4.02)
[WARN] [1781267991.210357853]: [DIAG wedge]   zone2 A[4.500,5.150] B[2.450,3.125] @(0.69,3.69)
[WARN] [1781267991.210366038]: [DIAG wedge] V2(s=0.007,rem=4.892,wait=17.3,act=0) vs V3(s=0.000,rem=4.461,wait=17.3,act=0) owner=V2 nzones=1 | A region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.450] stopline=-0.179 committed=0 | front_ext=0.179
[WARN] [1781267991.210404883]: [DIAG wedge]   zone0 A[0.000,4.875] B[0.000,4.450] @(2.05,4.26)
[WARN] [1781267991.210419314]: [DIAG wedge] V4(s=3.211,rem=0.968,wait=0.0,act=3) vs V6(s=0.013,rem=3.709,wait=17.3,act=0) owner=V4 nzones=1 | A region[se=0.000 sx=4.175] stopline=-0.179 committed=1 | B region[se=0.000 sx=3.600] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.210459825]: [DIAG wedge]   zone0 A[0.000,4.175] B[0.000,3.600] @(0.26,1.43)
[WARN] [1781267991.210490109]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=17.3,act=0) vs V6(s=0.013,rem=3.709,wait=17.3,act=0) owner=V5 nzones=2 | A region[se=1.225 sx=2.675] stopline=1.046 committed=0 | B region[se=1.025 sx=2.475] stopline=0.846 committed=0 | front_ext=0.179
[WARN] [1781267991.210531158]: [DIAG wedge]   zone0 A[1.225,1.900] B[1.825,2.475] @(1.26,1.32)
[WARN] [1781267991.210572654]: [DIAG wedge]   zone1 A[2.025,2.675] B[1.025,1.700] @(1.17,0.64)
[WARN] [1781267991.210585532]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=17.3,act=0) vs V7(s=1.174,rem=2.528,wait=0.0,act=3) owner=V7 nzones=1 | A region[se=0.000 sx=4.150] stopline=-0.179 committed=0 | B region[se=0.000 sx=3.575] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.210612627]: [DIAG wedge]   zone0 A[0.000,4.150] B[0.000,3.575] @(2.24,0.83)
[WARN] [1781267991.210656154]: [DIAG wedge] V6(s=0.013,rem=3.709,wait=17.3,act=0) vs V7(s=1.174,rem=2.528,wait=0.0,act=3) owner=V7 nzones=2 | A region[se=1.025 sx=2.475] stopline=0.846 committed=0 | B region[se=1.000 sx=2.475] stopline=0.821 committed=1 | front_ext=0.179
[WARN] [1781267991.210702575]: [DIAG wedge]   zone0 A[1.025,1.700] B[1.800,2.475] @(1.24,0.95)
[WARN] [1781267991.210714366]: [DIAG wedge]   zone1 A[1.825,2.475] B[1.000,1.675] @(1.33,1.62)
[WARN] [1781267991.212098506]: [DIAG wedge] V0(s=3.774,rem=1.564,wait=0.0,act=3) vs V2(s=0.007,rem=4.892,wait=20.3,act=0) owner=V0 nzones=1 | A region[se=0.000 sx=5.325] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.212141901]: [DIAG wedge]   zone0 A[0.000,5.325] B[0.000,4.875] @(2.27,4.26)
[WARN] [1781267991.212156210]: [DIAG wedge] V0(s=3.774,rem=1.564,wait=0.0,act=3) vs V3(s=0.000,rem=4.461,wait=20.3,act=0) owner=V0 nzones=1 | A region[se=0.650 sx=4.725] stopline=0.471 committed=1 | B region[se=0.325 sx=4.150] stopline=0.146 committed=0 | front_ext=0.179
[WARN] [1781267991.212163157]: [DIAG wedge]   zone0 A[0.650,4.725] B[0.325,4.150] @(1.96,3.85)
[WARN] [1781267991.212170733]: [DIAG wedge] V1(s=1.440,rem=3.716,wait=0.0,act=3) vs V2(s=0.007,rem=4.892,wait=20.3,act=0) owner=V1 nzones=3 | A region[se=0.575 sx=5.150] stopline=0.396 committed=1 | B region[se=0.375 sx=4.800] stopline=0.196 committed=0 | front_ext=0.179
[WARN] [1781267991.212179578]: [DIAG wedge]   zone0 A[0.575,2.550] B[0.375,2.325] @(2.06,2.99)
[WARN] [1781267991.212217347]: [DIAG wedge]   zone1 A[2.975,3.625] B[4.150,4.800] @(1.98,3.98)
[WARN] [1781267991.212258833]: [DIAG wedge]   zone2 A[4.500,5.150] B[2.650,3.325] @(0.69,3.69)
[WARN] [1781267991.212273416]: [DIAG wedge] V1(s=1.440,rem=3.716,wait=0.0,act=3) vs V3(s=0.000,rem=4.461,wait=20.3,act=0) owner=V1 nzones=3 | A region[se=0.575 sx=5.150] stopline=0.396 committed=1 | B region[se=0.325 sx=4.425] stopline=0.146 committed=0 | front_ext=0.179
[WARN] [1781267991.212299415]: [DIAG wedge]   zone0 A[0.575,2.425] B[0.325,2.125] @(1.94,2.91)
[WARN] [1781267991.212339052]: [DIAG wedge]   zone1 A[3.175,3.825] B[3.825,4.425] @(1.80,4.02)
[WARN] [1781267991.212380193]: [DIAG wedge]   zone2 A[4.500,5.150] B[2.450,3.125] @(0.69,3.69)
[WARN] [1781267991.212394238]: [DIAG wedge] V2(s=0.007,rem=4.892,wait=20.3,act=0) vs V3(s=0.000,rem=4.461,wait=20.3,act=0) owner=V2 nzones=1 | A region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.450] stopline=-0.179 committed=0 | front_ext=0.179
[WARN] [1781267991.212401469]: [DIAG wedge]   zone0 A[0.000,4.875] B[0.000,4.450] @(2.05,4.26)
[WARN] [1781267991.212408984]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=20.3,act=0) vs V6(s=0.245,rem=3.477,wait=0.0,act=3) owner=V5 nzones=2 | A region[se=1.225 sx=2.675] stopline=1.046 committed=0 | B region[se=1.025 sx=2.475] stopline=0.846 committed=0 | front_ext=0.179
[WARN] [1781267991.212447392]: [DIAG wedge]   zone0 A[1.225,1.900] B[1.825,2.475] @(1.26,1.32)
[WARN] [1781267991.212459173]: [DIAG wedge]   zone1 A[2.025,2.675] B[1.025,1.700] @(1.17,0.64)
[WARN] [1781267991.212468932]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=20.3,act=0) vs V7(s=1.738,rem=1.964,wait=0.0,act=3) owner=V7 nzones=1 | A region[se=0.000 sx=4.150] stopline=-0.179 committed=0 | B region[se=0.000 sx=3.575] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.212507717]: [DIAG wedge]   zone0 A[0.000,4.150] B[0.000,3.575] @(2.24,0.83)
[WARN] [1781267991.213651382]: [DIAG wedge] V0(s=4.374,rem=0.964,wait=0.0,act=3) vs V2(s=0.007,rem=4.892,wait=23.3,act=0) owner=V0 nzones=1 | A region[se=0.000 sx=5.325] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.213694807]: [DIAG wedge]   zone0 A[0.000,5.325] B[0.000,4.875] @(2.27,4.26)
[WARN] [1781267991.213724878]: [DIAG wedge] V1(s=2.040,rem=3.116,wait=0.0,act=3) vs V2(s=0.007,rem=4.892,wait=23.3,act=0) owner=V1 nzones=3 | A region[se=0.575 sx=5.150] stopline=0.396 committed=1 | B region[se=0.375 sx=4.800] stopline=0.196 committed=0 | front_ext=0.179
[WARN] [1781267991.213770386]: [DIAG wedge]   zone0 A[0.575,2.550] B[0.375,2.325] @(2.06,2.99)
[WARN] [1781267991.213811546]: [DIAG wedge]   zone1 A[2.975,3.625] B[4.150,4.800] @(1.98,3.98)
[WARN] [1781267991.213821753]: [DIAG wedge]   zone2 A[4.500,5.150] B[2.650,3.325] @(0.69,3.69)
[WARN] [1781267991.213830314]: [DIAG wedge] V1(s=2.040,rem=3.116,wait=0.0,act=3) vs V3(s=0.000,rem=4.461,wait=23.3,act=0) owner=V1 nzones=3 | A region[se=0.575 sx=5.150] stopline=0.396 committed=1 | B region[se=0.325 sx=4.425] stopline=0.146 committed=0 | front_ext=0.179
[WARN] [1781267991.213869636]: [DIAG wedge]   zone0 A[0.575,2.425] B[0.325,2.125] @(1.94,2.91)
[WARN] [1781267991.213883885]: [DIAG wedge]   zone1 A[3.175,3.825] B[3.825,4.425] @(1.80,4.02)
[WARN] [1781267991.213922039]: [DIAG wedge]   zone2 A[4.500,5.150] B[2.450,3.125] @(0.69,3.69)
[WARN] [1781267991.213935374]: [DIAG wedge] V2(s=0.007,rem=4.892,wait=23.3,act=0) vs V3(s=0.000,rem=4.461,wait=23.3,act=0) owner=V2 nzones=1 | A region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.450] stopline=-0.179 committed=0 | front_ext=0.179
[WARN] [1781267991.213962804]: [DIAG wedge]   zone0 A[0.000,4.875] B[0.000,4.450] @(2.05,4.26)
[WARN] [1781267991.214005651]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=23.3,act=0) vs V6(s=0.523,rem=3.199,wait=0.0,act=3) owner=V5 nzones=2 | A region[se=1.225 sx=2.675] stopline=1.046 committed=0 | B region[se=1.025 sx=2.475] stopline=0.846 committed=0 | front_ext=0.179
[WARN] [1781267991.214047725]: [DIAG wedge]   zone0 A[1.225,1.900] B[1.825,2.475] @(1.26,1.32)
[WARN] [1781267991.214074750]: [DIAG wedge]   zone1 A[2.025,2.675] B[1.025,1.700] @(1.17,0.64)
[WARN] [1781267991.214116642]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=23.3,act=0) vs V7(s=2.338,rem=1.364,wait=0.0,act=3) owner=V7 nzones=1 | A region[se=0.000 sx=4.150] stopline=-0.179 committed=0 | B region[se=0.000 sx=3.575] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.214160443]: [DIAG wedge]   zone0 A[0.000,4.150] B[0.000,3.575] @(2.24,0.83)
[ERROR] [1781267991.214926735]: [FIRST-WEDGE] @tick=250 sim_t=25.0s 最久=V3 wait=25.0s —— 回放前 80 拍历史 + 卡死车几何 ===
[WARN] [1781267991.214973086]: [WHIST]
tick=171
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.153/5.338 rem=2.185 spd=0.162 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=0.800/5.156 rem=4.356 spd=0.194 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=17.1 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=17.1 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.171/4.179 rem=1.008 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=17.1 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=17.1 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.134/3.702 rem=2.568 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.215009280]: [WHIST]
tick=172
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.169/5.338 rem=2.169 spd=0.162 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=0.820/5.156 rem=4.336 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=17.2 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=17.2 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.191/4.179 rem=0.988 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=17.2 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=17.2 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.154/3.702 rem=2.548 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.215053813]: [WHIST]
tick=173
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.185/5.338 rem=2.153 spd=0.163 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=0.840/5.156 rem=4.316 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=17.3 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=17.3 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.211/4.179 rem=0.968 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=17.3 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=17.3 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.174/3.702 rem=2.528 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.215099371]: [WHIST]
tick=174
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.202/5.338 rem=2.136 spd=0.166 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=0.860/5.156 rem=4.296 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=17.4 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=17.4 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.231/4.179 rem=0.948 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=17.4 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=17.4 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.194/3.702 rem=2.508 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.215132427]: [WHIST]
tick=175
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.219/5.338 rem=2.119 spd=0.170 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=0.880/5.156 rem=4.276 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=17.5 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=17.5 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.251/4.179 rem=0.928 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=17.5 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=17.5 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.214/3.702 rem=2.488 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.215177914]: [WHIST]
tick=176
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.236/5.338 rem=2.102 spd=0.176 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=0.900/5.156 rem=4.256 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=17.6 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=17.6 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.271/4.179 rem=0.908 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=17.6 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=17.6 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.234/3.702 rem=2.468 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.215209194]: [WHIST]
tick=177
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.255/5.338 rem=2.083 spd=0.183 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=0.920/5.156 rem=4.236 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=17.7 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=17.7 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.291/4.179 rem=0.888 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=17.7 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=17.7 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.254/3.702 rem=2.448 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.215254772]: [WHIST]
tick=178
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.274/5.338 rem=2.064 spd=0.192 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=0.940/5.156 rem=4.216 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=17.8 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=17.8 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.311/4.179 rem=0.868 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=17.8 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=17.8 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.274/3.702 rem=2.428 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.215286244]: [WHIST]
tick=179
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.294/5.338 rem=2.044 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=0.960/5.156 rem=4.196 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=17.9 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=17.9 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.331/4.179 rem=0.848 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=17.9 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=17.9 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.294/3.702 rem=2.408 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.215334209]: [WHIST]
tick=180
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.314/5.338 rem=2.024 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=0.980/5.156 rem=4.176 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=18.0 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=18.0 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.351/4.179 rem=0.828 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=18.0 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=18.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.314/3.702 rem=2.388 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.215368992]: [WHIST]
tick=181
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.334/5.338 rem=2.004 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.000/5.156 rem=4.156 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=18.1 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=18.1 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.371/4.179 rem=0.808 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=18.1 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=18.1 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.334/3.702 rem=2.368 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.215414652]: [WHIST]
tick=182
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.354/5.338 rem=1.984 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.020/5.156 rem=4.136 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=18.2 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=18.2 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.391/4.179 rem=0.788 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=18.2 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=18.2 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.354/3.702 rem=2.348 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.215446053]: [WHIST]
tick=183
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.374/5.338 rem=1.964 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.040/5.156 rem=4.116 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=18.3 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=18.3 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.411/4.179 rem=0.768 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=18.3 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=18.3 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.374/3.702 rem=2.329 spd=0.194 wait=0.0 gen=1
[WARN] [1781267991.215494079]: [WHIST]
tick=184
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.394/5.338 rem=1.944 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.060/5.156 rem=4.096 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=18.4 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=18.4 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.431/4.179 rem=0.748 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=18.4 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=18.4 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.392/3.702 rem=2.310 spd=0.187 wait=0.0 gen=1
[WARN] [1781267991.215525430]: [WHIST]
tick=185
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.414/5.338 rem=1.924 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.080/5.156 rem=4.076 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=18.5 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=18.5 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.451/4.179 rem=0.728 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=18.5 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.013/3.722 rem=3.709 spd=0.000 wait=18.5 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.410/3.702 rem=2.292 spd=0.182 wait=0.0 gen=1
[WARN] [1781267991.215572470]: [WHIST]
tick=186
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.434/5.338 rem=1.904 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.100/5.156 rem=4.056 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=18.6 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=18.6 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.471/4.179 rem=0.708 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=18.6 gen=1
  V6 mode=1 act=1 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.015/3.722 rem=3.707 spd=0.020 wait=18.6 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.428/3.702 rem=2.274 spd=0.177 wait=0.0 gen=1
[WARN] [1781267991.215605385]: [WHIST]
tick=187
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.454/5.338 rem=1.884 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.120/5.156 rem=4.036 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=18.7 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=18.7 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.491/4.179 rem=0.688 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=18.7 gen=1
  V6 mode=1 act=1 reason=action_hold blk=-1 brkr=0 task=0 slot=57->42 s=0.019/3.722 rem=3.703 spd=0.040 wait=18.7 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.445/3.702 rem=2.257 spd=0.172 wait=0.0 gen=1
[WARN] [1781267991.215652019]: [WHIST]
tick=188
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.474/5.338 rem=1.864 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.140/5.156 rem=4.016 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=18.8 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=18.8 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.511/4.179 rem=0.668 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=18.8 gen=1
  V6 mode=1 act=1 reason=action_hold blk=-1 brkr=0 task=0 slot=57->42 s=0.024/3.722 rem=3.698 spd=0.050 wait=18.8 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.462/3.702 rem=2.240 spd=0.168 wait=0.0 gen=1
[WARN] [1781267991.215683228]: [WHIST]
tick=189
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.494/5.338 rem=1.844 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.160/5.156 rem=3.996 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=18.9 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=18.9 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.531/4.179 rem=0.648 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=18.9 gen=1
  V6 mode=1 act=1 reason=action_hold blk=-1 brkr=0 task=0 slot=57->42 s=0.029/3.722 rem=3.693 spd=0.050 wait=18.9 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.479/3.702 rem=2.224 spd=0.165 wait=0.0 gen=1
[WARN] [1781267991.215728776]: [WHIST]
tick=190
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.514/5.338 rem=1.824 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.180/5.156 rem=3.976 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=19.0 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=19.0 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.551/4.179 rem=0.628 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=19.0 gen=1
  V6 mode=1 act=1 reason=action_hold blk=-1 brkr=0 task=0 slot=57->42 s=0.034/3.722 rem=3.688 spd=0.050 wait=19.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.495/3.702 rem=2.207 spd=0.164 wait=0.0 gen=1
[WARN] [1781267991.215759760]: [WHIST]
tick=191
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.534/5.338 rem=1.804 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.200/5.156 rem=3.956 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=19.1 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=19.1 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.571/4.179 rem=0.608 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=19.1 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.041/3.722 rem=3.681 spd=0.070 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.512/3.702 rem=2.191 spd=0.165 wait=0.0 gen=1
[WARN] [1781267991.215807146]: [WHIST]
tick=192
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.554/5.338 rem=1.784 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.220/5.156 rem=3.936 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=19.2 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=19.2 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.591/4.179 rem=0.588 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=19.2 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.050/3.722 rem=3.672 spd=0.090 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.528/3.702 rem=2.174 spd=0.167 wait=0.0 gen=1
[WARN] [1781267991.215838365]: [WHIST]
tick=193
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.574/5.338 rem=1.764 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.240/5.156 rem=3.916 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=19.3 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=19.3 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.611/4.179 rem=0.568 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=19.3 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.061/3.722 rem=3.661 spd=0.110 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.545/3.702 rem=2.157 spd=0.171 wait=0.0 gen=1
[WARN] [1781267991.215903361]: [WHIST]
tick=194
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.594/5.338 rem=1.744 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.260/5.156 rem=3.896 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=19.4 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=19.4 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.631/4.179 rem=0.548 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=19.4 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.074/3.722 rem=3.648 spd=0.130 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.563/3.702 rem=2.139 spd=0.175 wait=0.0 gen=1
[WARN] [1781267991.215934793]: [WHIST]
tick=195
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.614/5.338 rem=1.724 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.280/5.156 rem=3.876 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=19.5 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=19.5 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.651/4.179 rem=0.528 spd=0.195 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=19.5 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.089/3.722 rem=3.633 spd=0.150 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.581/3.702 rem=2.121 spd=0.180 wait=0.0 gen=1
[WARN] [1781267991.215980503]: [WHIST]
tick=196
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.634/5.338 rem=1.704 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.300/5.156 rem=3.856 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=19.6 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=19.6 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.670/4.179 rem=0.510 spd=0.188 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=19.6 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.106/3.722 rem=3.616 spd=0.170 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.600/3.702 rem=2.103 spd=0.185 wait=0.0 gen=1
[WARN] [1781267991.216011752]: [WHIST]
tick=197
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.654/5.338 rem=1.684 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.320/5.156 rem=3.836 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=19.7 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=19.7 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.688/4.179 rem=0.491 spd=0.182 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=19.7 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.125/3.722 rem=3.597 spd=0.190 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.619/3.702 rem=2.084 spd=0.191 wait=0.0 gen=1
[WARN] [1781267991.216058498]: [WHIST]
tick=198
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.674/5.338 rem=1.664 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.340/5.156 rem=3.816 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=19.8 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=19.8 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.706/4.179 rem=0.474 spd=0.177 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=19.8 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.145/3.722 rem=3.577 spd=0.200 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.638/3.702 rem=2.064 spd=0.198 wait=0.0 gen=1
[WARN] [1781267991.216089544]: [WHIST]
tick=199
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.694/5.338 rem=1.644 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.360/5.156 rem=3.796 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=19.9 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=19.9 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.723/4.179 rem=0.456 spd=0.172 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=19.9 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.165/3.722 rem=3.557 spd=0.200 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.658/3.702 rem=2.044 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216136331]: [WHIST]
tick=200
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.714/5.338 rem=1.624 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.380/5.156 rem=3.776 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=20.0 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=20.0 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.740/4.179 rem=0.440 spd=0.168 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=20.0 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.185/3.722 rem=3.537 spd=0.200 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.678/3.702 rem=2.024 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216167539]: [WHIST]
tick=201
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.734/5.338 rem=1.604 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.400/5.156 rem=3.756 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=20.1 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=20.1 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.756/4.179 rem=0.423 spd=0.165 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=20.1 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.205/3.722 rem=3.517 spd=0.200 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.698/3.702 rem=2.004 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216214539]: [WHIST]
tick=202
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.754/5.338 rem=1.584 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.420/5.156 rem=3.736 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=20.2 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=20.2 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.773/4.179 rem=0.407 spd=0.164 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=20.2 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.225/3.722 rem=3.497 spd=0.200 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.718/3.702 rem=1.984 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216245605]: [WHIST]
tick=203
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.774/5.338 rem=1.564 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.440/5.156 rem=3.716 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=20.3 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=20.3 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.789/4.179 rem=0.390 spd=0.164 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=20.3 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.245/3.722 rem=3.477 spd=0.200 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.738/3.702 rem=1.964 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216293235]: [WHIST]
tick=204
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.794/5.338 rem=1.544 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.460/5.156 rem=3.696 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=20.4 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=20.4 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.806/4.179 rem=0.374 spd=0.165 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=20.4 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.265/3.722 rem=3.457 spd=0.200 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.758/3.702 rem=1.944 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216324291]: [WHIST]
tick=205
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.814/5.338 rem=1.524 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.480/5.156 rem=3.676 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=20.5 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=20.5 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.822/4.179 rem=0.357 spd=0.168 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=20.5 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.285/3.722 rem=3.437 spd=0.200 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.778/3.702 rem=1.924 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216371170]: [WHIST]
tick=206
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.834/5.338 rem=1.504 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.500/5.156 rem=3.656 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=20.6 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=20.6 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.840/4.179 rem=0.340 spd=0.172 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=20.6 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.305/3.722 rem=3.417 spd=0.200 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.798/3.702 rem=1.904 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216402195]: [WHIST]
tick=207
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.854/5.338 rem=1.484 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.520/5.156 rem=3.636 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=20.7 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=20.7 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.857/4.179 rem=0.322 spd=0.177 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=20.7 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.325/3.722 rem=3.397 spd=0.200 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.818/3.702 rem=1.884 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216449317]: [WHIST]
tick=208
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.874/5.338 rem=1.464 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.540/5.156 rem=3.616 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=20.8 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=20.8 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.876/4.179 rem=0.304 spd=0.182 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=20.8 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.345/3.722 rem=3.377 spd=0.200 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.838/3.702 rem=1.864 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216483338]: [WHIST]
tick=209
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.894/5.338 rem=1.444 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.560/5.156 rem=3.596 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=20.9 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=20.9 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.894/4.179 rem=0.285 spd=0.188 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=20.9 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.365/3.722 rem=3.357 spd=0.200 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.858/3.702 rem=1.844 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216529029]: [WHIST]
tick=210
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.914/5.338 rem=1.424 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.580/5.156 rem=3.576 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=21.0 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=21.0 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.914/4.179 rem=0.266 spd=0.194 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=21.0 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.385/3.722 rem=3.337 spd=0.200 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.878/3.702 rem=1.824 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216562664]: [WHIST]
tick=211
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.934/5.338 rem=1.404 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.600/5.156 rem=3.556 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=21.1 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=21.1 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.934/4.179 rem=0.246 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=21.1 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.405/3.722 rem=3.317 spd=0.200 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.898/3.702 rem=1.804 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216609522]: [WHIST]
tick=212
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.954/5.338 rem=1.384 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.620/5.156 rem=3.536 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=21.2 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=21.2 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.954/4.179 rem=0.226 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=21.2 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.422/3.722 rem=3.300 spd=0.170 wait=0.1 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.918/3.702 rem=1.784 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216641045]: [WHIST]
tick=213
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.974/5.338 rem=1.364 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.640/5.156 rem=3.516 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=21.3 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=21.3 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.974/4.179 rem=0.206 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=21.3 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.436/3.722 rem=3.286 spd=0.140 wait=0.2 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.938/3.702 rem=1.764 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216688472]: [WHIST]
tick=214
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=3.994/5.338 rem=1.344 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.660/5.156 rem=3.496 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=21.4 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=21.4 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=3.994/4.179 rem=0.186 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=21.4 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.447/3.722 rem=3.275 spd=0.110 wait=0.3 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.958/3.702 rem=1.744 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216721122]: [WHIST]
tick=215
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.014/5.338 rem=1.324 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.680/5.156 rem=3.476 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=21.5 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=21.5 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=4.014/4.179 rem=0.166 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=21.5 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.455/3.722 rem=3.267 spd=0.080 wait=0.4 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.978/3.702 rem=1.724 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216769179]: [WHIST]
tick=216
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.034/5.338 rem=1.304 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.700/5.156 rem=3.456 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=21.6 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=21.6 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=4.034/4.179 rem=0.146 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=21.6 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.460/3.722 rem=3.262 spd=0.050 wait=0.5 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=1.998/3.702 rem=1.704 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216800367]: [WHIST]
tick=217
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.054/5.338 rem=1.284 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.720/5.156 rem=3.436 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=21.7 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=21.7 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=4.054/4.179 rem=0.126 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=21.7 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.462/3.722 rem=3.260 spd=0.020 wait=0.6 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.018/3.702 rem=1.684 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216847438]: [WHIST]
tick=218
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.074/5.338 rem=1.264 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.740/5.156 rem=3.416 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=21.8 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=21.8 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=4.074/4.179 rem=0.106 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=21.8 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.462/3.722 rem=3.260 spd=0.000 wait=0.7 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.038/3.702 rem=1.664 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216888355]: [WHIST]
tick=219
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.094/5.338 rem=1.244 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.760/5.156 rem=3.396 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=21.9 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=21.9 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=4.094/4.179 rem=0.086 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=21.9 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.462/3.722 rem=3.260 spd=0.000 wait=0.8 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.058/3.702 rem=1.644 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216934228]: [WHIST]
tick=220
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.114/5.338 rem=1.224 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.780/5.156 rem=3.376 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=22.0 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=22.0 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=4.114/4.179 rem=0.066 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=22.0 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.462/3.722 rem=3.260 spd=0.000 wait=0.9 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.078/3.702 rem=1.624 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.216969183]: [WHIST]
tick=221
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.134/5.338 rem=1.204 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.800/5.156 rem=3.356 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=22.1 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=22.1 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=4.134/4.179 rem=0.046 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=22.1 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.462/3.722 rem=3.260 spd=0.000 wait=1.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.098/3.702 rem=1.604 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.217014010]: [WHIST]
tick=222
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.154/5.338 rem=1.184 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.820/5.156 rem=3.336 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=22.2 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=22.2 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=4.154/4.179 rem=0.026 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=22.2 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.462/3.722 rem=3.260 spd=0.000 wait=1.1 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.118/3.702 rem=1.584 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.217045513]: [WHIST]
tick=223
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.174/5.338 rem=1.164 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.840/5.156 rem=3.316 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=22.3 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=22.3 gen=1
  V4 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=56->40 s=4.174/4.179 rem=0.006 spd=0.200 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=22.3 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.462/3.722 rem=3.260 spd=0.000 wait=1.2 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.138/3.702 rem=1.564 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.217092168]: [WHIST]
tick=224
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.194/5.338 rem=1.144 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.860/5.156 rem=3.296 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=22.4 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=22.4 gen=1
  V4 mode=2 act=0 reason=dwell blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=22.4 gen=1
  V6 mode=1 act=0 reason=brake_V4 blk=4 brkr=0 task=0 slot=57->42 s=0.462/3.722 rem=3.260 spd=0.000 wait=1.3 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.158/3.702 rem=1.544 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.217123569]: [WHIST]
tick=225
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.214/5.338 rem=1.124 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.880/5.156 rem=3.276 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=22.5 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=22.5 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=22.5 gen=1
  V6 mode=1 act=1 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.464/3.722 rem=3.258 spd=0.020 wait=1.4 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.178/3.702 rem=1.524 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.217169148]: [WHIST]
tick=226
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.234/5.338 rem=1.104 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.900/5.156 rem=3.256 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=22.6 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=22.6 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=22.6 gen=1
  V6 mode=1 act=1 reason=action_hold blk=-1 brkr=0 task=0 slot=57->42 s=0.468/3.722 rem=3.254 spd=0.040 wait=1.5 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.198/3.702 rem=1.504 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.217200955]: [WHIST]
tick=227
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.254/5.338 rem=1.084 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.920/5.156 rem=3.236 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=22.7 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=22.7 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=22.7 gen=1
  V6 mode=1 act=1 reason=action_hold blk=-1 brkr=0 task=0 slot=57->42 s=0.473/3.722 rem=3.249 spd=0.050 wait=1.6 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.218/3.702 rem=1.484 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.217247894]: [WHIST]
tick=228
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.274/5.338 rem=1.064 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.940/5.156 rem=3.216 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=22.8 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=22.8 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=22.8 gen=1
  V6 mode=1 act=1 reason=action_hold blk=-1 brkr=0 task=0 slot=57->42 s=0.478/3.722 rem=3.244 spd=0.050 wait=1.7 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.238/3.702 rem=1.464 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.217296093]: [WHIST]
tick=229
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.294/5.338 rem=1.044 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.960/5.156 rem=3.196 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=22.9 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=22.9 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=22.9 gen=1
  V6 mode=1 act=1 reason=action_hold blk=-1 brkr=0 task=0 slot=57->42 s=0.483/3.722 rem=3.239 spd=0.050 wait=1.8 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.258/3.702 rem=1.444 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.217345124]: [WHIST]
tick=230
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.314/5.338 rem=1.024 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=1.980/5.156 rem=3.176 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=23.0 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=23.0 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=23.0 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.490/3.722 rem=3.232 spd=0.070 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.278/3.702 rem=1.424 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.217379796]: [WHIST]
tick=231
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.334/5.338 rem=1.004 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.000/5.156 rem=3.156 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=23.1 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=23.1 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=23.1 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.499/3.722 rem=3.223 spd=0.090 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.298/3.702 rem=1.404 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.217424491]: [WHIST]
tick=232
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.354/5.338 rem=0.984 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.020/5.156 rem=3.136 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=23.2 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=23.2 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=23.2 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.510/3.722 rem=3.212 spd=0.110 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.318/3.702 rem=1.384 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.217471795]: [WHIST]
tick=233
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.374/5.338 rem=0.964 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.040/5.156 rem=3.116 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=23.3 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=23.3 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=23.3 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.523/3.722 rem=3.199 spd=0.130 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.338/3.702 rem=1.364 spd=0.194 wait=0.0 gen=1
[WARN] [1781267991.217505075]: [WHIST]
tick=234
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.394/5.338 rem=0.944 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.060/5.156 rem=3.096 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=23.4 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=23.4 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=23.4 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.538/3.722 rem=3.184 spd=0.150 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.357/3.702 rem=1.346 spd=0.188 wait=0.0 gen=1
[WARN] [1781267991.217552695]: [WHIST]
tick=235
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.414/5.338 rem=0.924 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.080/5.156 rem=3.076 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=23.5 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=23.5 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=23.5 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.554/3.722 rem=3.168 spd=0.162 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.375/3.702 rem=1.327 spd=0.182 wait=0.0 gen=1
[WARN] [1781267991.217583731]: [WHIST]
tick=236
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.434/5.338 rem=0.904 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.100/5.156 rem=3.056 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=23.6 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=23.6 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=23.6 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.570/3.722 rem=3.151 spd=0.162 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.393/3.702 rem=1.310 spd=0.177 wait=0.0 gen=1
[WARN] [1781267991.217631137]: [WHIST]
tick=237
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.454/5.338 rem=0.884 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.120/5.156 rem=3.036 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=23.7 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=23.7 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=23.7 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.587/3.722 rem=3.135 spd=0.162 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.410/3.702 rem=1.292 spd=0.173 wait=0.0 gen=1
[WARN] [1781267991.217676634]: [WHIST]
tick=238
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.474/5.338 rem=0.864 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.140/5.156 rem=3.016 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=23.8 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=23.8 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=23.8 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.603/3.722 rem=3.119 spd=0.162 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.427/3.702 rem=1.276 spd=0.169 wait=0.0 gen=1
[WARN] [1781267991.217692203]: [WHIST]
tick=239
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.494/5.338 rem=0.844 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.160/5.156 rem=2.996 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=23.9 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=23.9 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=23.9 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.619/3.722 rem=3.103 spd=0.162 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.443/3.702 rem=1.259 spd=0.166 wait=0.0 gen=1
[WARN] [1781267991.217734948]: [WHIST]
tick=240
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.514/5.338 rem=0.824 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.180/5.156 rem=2.976 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=24.0 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=24.0 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=24.0 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.635/3.722 rem=3.087 spd=0.162 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.460/3.702 rem=1.243 spd=0.164 wait=0.0 gen=1
[WARN] [1781267991.217780120]: [WHIST]
tick=241
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.534/5.338 rem=0.804 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.200/5.156 rem=2.956 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=24.1 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=24.1 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=24.1 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.651/3.722 rem=3.070 spd=0.162 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.476/3.702 rem=1.226 spd=0.165 wait=0.0 gen=1
[WARN] [1781267991.217812953]: [WHIST]
tick=242
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.554/5.338 rem=0.784 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.220/5.156 rem=2.936 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=24.2 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=24.2 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=24.2 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.667/3.722 rem=3.054 spd=0.162 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.493/3.702 rem=1.209 spd=0.167 wait=0.0 gen=1
[WARN] [1781267991.217856491]: [WHIST]
tick=243
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.574/5.338 rem=0.764 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.240/5.156 rem=2.916 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=0 reason=brake_V0 blk=0 brkr=0 task=0 slot=36->8 s=0.007/4.899 rem=4.892 spd=0.000 wait=24.3 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=24.3 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=24.3 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.684/3.722 rem=3.038 spd=0.162 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.510/3.702 rem=1.192 spd=0.171 wait=0.0 gen=1
[WARN] [1781267991.217887313]: [WHIST]
tick=244
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.594/5.338 rem=0.744 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.260/5.156 rem=2.896 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=1 reason=clear blk=-1 brkr=0 task=0 slot=36->8 s=0.009/4.899 rem=4.890 spd=0.020 wait=24.4 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=24.4 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=24.4 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.700/3.722 rem=3.022 spd=0.162 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.527/3.702 rem=1.175 spd=0.175 wait=0.0 gen=1
[WARN] [1781267991.217918013]: [WHIST]
tick=245
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.614/5.338 rem=0.724 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.280/5.156 rem=2.876 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=1 reason=action_hold blk=-1 brkr=0 task=0 slot=36->8 s=0.013/4.899 rem=4.886 spd=0.040 wait=24.5 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=24.5 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=24.5 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.716/3.722 rem=3.006 spd=0.162 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.545/3.702 rem=1.157 spd=0.180 wait=0.0 gen=1
[WARN] [1781267991.217948866]: [WHIST]
tick=246
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.634/5.338 rem=0.704 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.300/5.156 rem=2.856 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=1 reason=action_hold blk=-1 brkr=0 task=0 slot=36->8 s=0.018/4.899 rem=4.881 spd=0.050 wait=24.6 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=24.6 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=24.6 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.732/3.722 rem=2.990 spd=0.162 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.564/3.702 rem=1.138 spd=0.185 wait=0.0 gen=1
[WARN] [1781267991.217994739]: [WHIST]
tick=247
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.654/5.338 rem=0.684 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.320/5.156 rem=2.836 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=1 reason=action_hold blk=-1 brkr=0 task=0 slot=36->8 s=0.023/4.899 rem=4.876 spd=0.050 wait=24.7 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=24.7 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=24.7 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.748/3.722 rem=2.973 spd=0.162 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.583/3.702 rem=1.119 spd=0.191 wait=0.0 gen=1
[WARN] [1781267991.218025551]: [WHIST]
tick=248
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.674/5.338 rem=0.664 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.340/5.156 rem=2.816 spd=0.200 wait=0.0 gen=1
  V2 mode=1 act=1 reason=action_hold blk=-1 brkr=0 task=0 slot=36->8 s=0.028/4.899 rem=4.871 spd=0.050 wait=24.8 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=24.8 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=24.8 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.764/3.722 rem=2.957 spd=0.162 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.603/3.702 rem=1.100 spd=0.197 wait=0.0 gen=1
[WARN] [1781267991.218070916]: [WHIST]
tick=249
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.694/5.338 rem=0.644 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.360/5.156 rem=2.796 spd=0.198 wait=0.0 gen=1
  V2 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=36->8 s=0.035/4.899 rem=4.864 spd=0.070 wait=0.0 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=24.9 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=24.9 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.781/3.722 rem=2.941 spd=0.163 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.623/3.702 rem=1.080 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.218101607]: [WHIST]
tick=250
  V0 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=38->9 s=4.714/5.338 rem=0.624 spd=0.200 wait=0.0 gen=1
  V1 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=20->10 s=2.379/5.156 rem=2.777 spd=0.191 wait=0.0 gen=1
  V2 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=36->8 s=0.044/4.899 rem=4.855 spd=0.090 wait=0.0 gen=1
  V3 mode=1 act=0 reason=brake_V2 blk=2 brkr=0 task=0 slot=34->7 s=0.000/4.461 rem=4.461 spd=0.000 wait=25.0 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=1 act=0 reason=brake_V7 blk=7 brkr=0 task=0 slot=39->55 s=0.000/4.160 rem=4.160 spd=0.000 wait=25.0 gen=1
  V6 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=57->42 s=0.798/3.722 rem=2.924 spd=0.172 wait=0.0 gen=1
  V7 mode=1 act=3 reason=clear blk=-1 brkr=0 task=0 slot=37->53 s=2.643/3.702 rem=1.060 spd=0.200 wait=0.0 gen=1
[WARN] [1781267991.218150394]: [CONFLICT] V3(s=0.000 rem=4.461 act=0 blk=2) vs V0(s=4.714 rem=0.624 act=3 blk=-1) | owner=V0 following=1 nzones=1 all_same_dir=1
[WARN] [1781267991.218178160]: [CONFLICT]   zone0 same_dir=1 | A[0.325,4.150] stopA=0.146 committedA=0 | B[0.650,4.725] stopB=0.471 committedB=1 | @(1.96,3.85)
[WARN] [1781267991.218219737]: [CONFLICT] V3(s=0.000 rem=4.461 act=0 blk=2) vs V1(s=2.379 rem=2.777 act=3 blk=-1) | owner=V1 following=0 nzones=3 all_same_dir=0
[WARN] [1781267991.218246893]: [CONFLICT]   zone0 same_dir=0 | A[0.325,2.125] stopA=0.146 committedA=0 | B[0.575,2.425] stopB=0.396 committedB=1 | @(1.94,2.91)
[WARN] [1781267991.218287790]: [CONFLICT]   zone1 same_dir=0 | A[3.825,4.425] stopA=3.646 committedA=0 | B[3.175,3.825] stopB=2.996 committedB=0 | @(1.80,4.02)
[WARN] [1781267991.218315007]: [CONFLICT]   zone2 same_dir=0 | A[2.450,3.125] stopA=2.271 committedA=0 | B[4.500,5.150] stopB=4.321 committedB=0 | @(0.69,3.69)
[WARN] [1781267991.218340467]: [CONFLICT] V3(s=0.000 rem=4.461 act=0 blk=2) vs V2(s=0.044 rem=4.855 act=3 blk=-1) | owner=V2 following=0 nzones=1 all_same_dir=1
[WARN] [1781267991.218367735]: [CONFLICT]   zone0 same_dir=1 | A[0.000,4.450] stopA=-0.179 committedA=0 | B[0.000,4.875] stopB=-0.179 committedB=1 | @(2.05,4.26)
[WARN] [1781267991.218408906]: [CONFLICT] V3(s=0.000 rem=4.461 act=0 blk=2) vs V4(s=4.179 rem=0.000 act=0 blk=-1) | owner=V-1 following=0 nzones=0 all_same_dir=1
[WARN] [1781267991.218436001]: [CONFLICT] V3(s=0.000 rem=4.461 act=0 blk=2) vs V5(s=0.000 rem=4.160 act=0 blk=7) | owner=V-1 following=0 nzones=0 all_same_dir=1
[WARN] [1781267991.218477284]: [CONFLICT] V3(s=0.000 rem=4.461 act=0 blk=2) vs V6(s=0.798 rem=2.924 act=3 blk=-1) | owner=V-1 following=0 nzones=0 all_same_dir=1
[WARN] [1781267991.218505770]: [CONFLICT] V3(s=0.000 rem=4.461 act=0 blk=2) vs V7(s=2.643 rem=1.060 act=3 blk=-1) | owner=V-1 following=0 nzones=0 all_same_dir=1
[WARN] [1781267991.218639469]: [DIAG wedge] V0(s=4.754,rem=0.584,wait=0.0,act=3) vs V3(s=0.000,rem=4.461,wait=25.2,act=0) owner=V0 nzones=1 | A region[se=0.650 sx=4.725] stopline=0.471 committed=1 | B region[se=0.325 sx=4.150] stopline=0.146 committed=0 | front_ext=0.179
[WARN] [1781267991.218681544]: [DIAG wedge]   zone0 A[0.650,4.725] B[0.325,4.150] @(1.96,3.85)
[WARN] [1781267991.219185009]: [DIAG wedge] V1(s=2.605,rem=2.551,wait=0.0,act=3) vs V3(s=0.000,rem=4.461,wait=26.3,act=0) owner=V1 nzones=2 | A region[se=3.175 sx=5.150] stopline=2.996 committed=0 | B region[se=2.450 sx=4.425] stopline=2.271 committed=0 | front_ext=0.179
[WARN] [1781267991.219226769]: [DIAG wedge]   zone0 A[3.175,3.825] B[3.825,4.425] @(1.80,4.02)
[WARN] [1781267991.219253224]: [DIAG wedge]   zone1 A[4.500,5.150] B[2.450,3.125] @(0.69,3.69)
[WARN] [1781267991.219280705]: [DIAG wedge] V2(s=0.118,rem=4.781,wait=0.0,act=3) vs V3(s=0.000,rem=4.461,wait=26.3,act=0) owner=V2 nzones=1 | A region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.450] stopline=-0.179 committed=0 | front_ext=0.179
[WARN] [1781267991.219309801]: [DIAG wedge]   zone0 A[0.000,4.875] B[0.000,4.450] @(2.05,4.26)
[WARN] [1781267991.219353592]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=26.3,act=0) vs V6(s=0.847,rem=2.875,wait=1.3,act=0) owner=V5 nzones=2 | A region[se=1.225 sx=2.675] stopline=1.046 committed=0 | B region[se=1.025 sx=2.475] stopline=0.846 committed=0 | front_ext=0.179
[WARN] [1781267991.219380322]: [DIAG wedge]   zone0 A[1.225,1.900] B[1.825,2.475] @(1.26,1.32)
[WARN] [1781267991.219420071]: [DIAG wedge]   zone1 A[2.025,2.675] B[1.025,1.700] @(1.17,0.64)
[WARN] [1781267991.219449745]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=26.3,act=0) vs V7(s=2.903,rem=0.800,wait=0.0,act=3) owner=V7 nzones=1 | A region[se=0.000 sx=4.150] stopline=-0.179 committed=0 | B region[se=0.000 sx=3.575] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.219491079]: [DIAG wedge]   zone0 A[0.000,4.150] B[0.000,3.575] @(2.24,0.83)
[WARN] [1781267991.220403155]: [DIAG wedge] V1(s=3.035,rem=2.121,wait=0.0,act=3) vs V3(s=0.000,rem=4.461,wait=29.3,act=0) owner=V1 nzones=2 | A region[se=3.175 sx=5.150] stopline=2.996 committed=0 | B region[se=2.450 sx=4.425] stopline=2.271 committed=0 | front_ext=0.179
[WARN] [1781267991.220445555]: [DIAG wedge]   zone0 A[3.175,3.825] B[3.825,4.425] @(1.80,4.02)
[WARN] [1781267991.220457305]: [DIAG wedge]   zone1 A[4.500,5.150] B[2.450,3.125] @(0.69,3.69)
[WARN] [1781267991.220466181]: [DIAG wedge] V2(s=0.329,rem=4.570,wait=0.0,act=3) vs V3(s=0.000,rem=4.461,wait=29.3,act=0) owner=V2 nzones=1 | A region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.450] stopline=-0.179 committed=0 | front_ext=0.179
[WARN] [1781267991.220505514]: [DIAG wedge]   zone0 A[0.000,4.875] B[0.000,4.450] @(2.05,4.26)
[WARN] [1781267991.220518960]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=29.3,act=0) vs V6(s=0.847,rem=2.875,wait=4.3,act=0) owner=V5 nzones=2 | A region[se=1.225 sx=2.675] stopline=1.046 committed=0 | B region[se=1.025 sx=2.475] stopline=0.846 committed=0 | front_ext=0.179
[WARN] [1781267991.220545080]: [DIAG wedge]   zone0 A[1.225,1.900] B[1.825,2.475] @(1.26,1.32)
[WARN] [1781267991.220586607]: [DIAG wedge]   zone1 A[2.025,2.675] B[1.025,1.700] @(1.17,0.64)
[WARN] [1781267991.220616281]: [DIAG wedge] V5(s=0.000,rem=4.160,wait=29.3,act=0) vs V7(s=3.465,rem=0.237,wait=0.0,act=3) owner=V7 nzones=1 | A region[se=0.000 sx=4.150] stopline=-0.179 committed=0 | B region[se=0.000 sx=3.575] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.220658021]: [DIAG wedge]   zone0 A[0.000,4.150] B[0.000,3.575] @(2.24,0.83)
[WARN] [1781267991.221707625]: [DIAG wedge] V1(s=3.600,rem=1.557,wait=0.0,act=3) vs V3(s=0.000,rem=4.461,wait=32.3,act=0) owner=V1 nzones=2 | A region[se=3.175 sx=5.150] stopline=2.996 committed=1 | B region[se=2.450 sx=4.425] stopline=2.271 committed=0 | front_ext=0.179
[WARN] [1781267991.221750035]: [DIAG wedge]   zone0 A[3.175,3.825] B[3.825,4.425] @(1.80,4.02)
[WARN] [1781267991.221776449]: [DIAG wedge]   zone1 A[4.500,5.150] B[2.450,3.125] @(0.69,3.69)
[WARN] [1781267991.222012862]: [DIAG wedge] V2(s=0.939,rem=3.961,wait=0.0,act=3) vs V3(s=0.006,rem=4.455,wait=32.8,act=1) owner=V2 nzones=1 | A region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.450] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.222054399]: [DIAG wedge]   zone0 A[0.000,4.875] B[0.000,4.450] @(2.05,4.26)
[WARN] [1781267991.222165867]: [DIAG wedge] V5(s=0.460,rem=3.700,wait=0.0,act=3) vs V6(s=0.847,rem=2.875,wait=8.1,act=0) owner=V5 nzones=2 | A region[se=1.225 sx=2.675] stopline=1.046 committed=0 | B region[se=1.025 sx=2.475] stopline=0.846 committed=0 | front_ext=0.179
[WARN] [1781267991.222204986]: [DIAG wedge]   zone0 A[1.225,1.900] B[1.825,2.475] @(1.26,1.32)
[WARN] [1781267991.222231157]: [DIAG wedge]   zone1 A[2.025,2.675] B[1.025,1.700] @(1.17,0.64)
[WARN] [1781267991.222932728]: [DIAG wedge] V1(s=4.200,rem=0.957,wait=0.0,act=3) vs V3(s=0.007,rem=4.454,wait=35.3,act=0) owner=V1 nzones=1 | A region[se=4.500 sx=5.150] stopline=4.321 committed=0 | B region[se=2.450 sx=3.125] stopline=2.271 committed=0 | front_ext=0.179
[WARN] [1781267991.222983994]: [DIAG wedge]   zone0 A[4.500,5.150] B[2.450,3.125] @(0.69,3.69)
[WARN] [1781267991.223186365]: [DIAG wedge] V2(s=1.539,rem=3.361,wait=0.0,act=3) vs V3(s=0.007,rem=4.454,wait=35.8,act=0) owner=V2 nzones=1 | A region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.450] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.223227718]: [DIAG wedge]   zone0 A[0.000,4.875] B[0.000,4.450] @(2.05,4.26)
[WARN] [1781267991.223340090]: [DIAG wedge] V5(s=0.985,rem=3.175,wait=0.0,act=3) vs V6(s=0.847,rem=2.875,wait=11.1,act=0) owner=V5 nzones=2 | A region[se=1.225 sx=2.675] stopline=1.046 committed=0 | B region[se=1.025 sx=2.475] stopline=0.846 committed=0 | front_ext=0.179
[WARN] [1781267991.223379718]: [DIAG wedge]   zone0 A[1.225,1.900] B[1.825,2.475] @(1.26,1.32)
[WARN] [1781267991.223406549]: [DIAG wedge]   zone1 A[2.025,2.675] B[1.025,1.700] @(1.17,0.64)
[WARN] [1781267991.224153017]: [DIAG wedge] V1(s=4.775,rem=0.381,wait=0.0,act=3) vs V3(s=0.007,rem=4.454,wait=38.3,act=0) owner=V1 nzones=1 | A region[se=4.500 sx=5.150] stopline=4.321 committed=1 | B region[se=2.450 sx=3.125] stopline=2.271 committed=0 | front_ext=0.179
[WARN] [1781267991.224196707]: [DIAG wedge]   zone0 A[4.500,5.150] B[2.450,3.125] @(0.69,3.69)
[WARN] [1781267991.224365382]: [DIAG wedge] V2(s=2.126,rem=2.773,wait=0.0,act=3) vs V3(s=0.007,rem=4.454,wait=38.8,act=0) owner=V2 nzones=1 | A region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.450] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.224404613]: [DIAG wedge]   zone0 A[0.000,4.875] B[0.000,4.450] @(2.05,4.26)
[WARN] [1781267991.224507794]: [DIAG wedge] V5(s=1.585,rem=2.575,wait=0.0,act=3) vs V6(s=0.847,rem=2.875,wait=14.1,act=0) owner=V5 nzones=2 | A region[se=1.225 sx=2.675] stopline=1.046 committed=1 | B region[se=1.025 sx=2.475] stopline=0.846 committed=0 | front_ext=0.179
[WARN] [1781267991.224547594]: [DIAG wedge]   zone0 A[1.225,1.900] B[1.825,2.475] @(1.26,1.32)
[WARN] [1781267991.224559364]: [DIAG wedge]   zone1 A[2.025,2.675] B[1.025,1.700] @(1.17,0.64)
[WARN] [1781267991.225283155]: [DIAG wedge] V2(s=2.684,rem=2.215,wait=0.0,act=3) vs V3(s=0.007,rem=4.454,wait=41.8,act=0) owner=V2 nzones=1 | A region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.450] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.225326500]: [DIAG wedge]   zone0 A[0.000,4.875] B[0.000,4.450] @(2.05,4.26)
[WARN] [1781267991.225415666]: [DIAG wedge] V5(s=2.149,rem=2.011,wait=0.0,act=3) vs V6(s=0.847,rem=2.875,wait=17.1,act=0) owner=V5 nzones=1 | A region[se=2.025 sx=2.675] stopline=1.846 committed=1 | B region[se=1.025 sx=1.700] stopline=0.846 committed=0 | front_ext=0.179
[WARN] [1781267991.225455375]: [DIAG wedge]   zone0 A[2.025,2.675] B[1.025,1.700] @(1.17,0.64)
[WARN] [1781267991.226109955]: [DIAG wedge] V2(s=3.230,rem=1.670,wait=0.0,act=3) vs V3(s=0.007,rem=4.454,wait=44.8,act=0) owner=V2 nzones=1 | A region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.450] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.226153127]: [DIAG wedge]   zone0 A[0.000,4.875] B[0.000,4.450] @(2.05,4.26)
[WARN] [1781267991.226962459]: [DIAG wedge] V2(s=3.830,rem=1.070,wait=0.0,act=3) vs V3(s=0.007,rem=4.454,wait=47.8,act=0) owner=V2 nzones=1 | A region[se=0.000 sx=4.875] stopline=-0.179 committed=1 | B region[se=0.000 sx=4.450] stopline=-0.179 committed=1 | front_ext=0.179
[WARN] [1781267991.227004259]: [DIAG wedge]   zone0 A[0.000,4.875] B[0.000,4.450] @(2.05,4.26)
[INFO] [1781267991.272712627]: [batch] 7200/72000 ticks (sim_t=720s) hard_guard=0
[INFO] [1781267991.317705349]: [batch] 14400/72000 ticks (sim_t=1440s) hard_guard=0
[INFO] [1781267991.362691134]: [batch] 21600/72000 ticks (sim_t=2160s) hard_guard=0
[INFO] [1781267991.407651946]: [batch] 28800/72000 ticks (sim_t=2880s) hard_guard=0
[INFO] [1781267991.452542126]: [batch] 36000/72000 ticks (sim_t=3600s) hard_guard=0
[INFO] [1781267991.497586977]: [batch] 43200/72000 ticks (sim_t=4320s) hard_guard=0
[INFO] [1781267991.542521080]: [batch] 50400/72000 ticks (sim_t=5040s) hard_guard=0
[INFO] [1781267991.587418999]: [batch] 57600/72000 ticks (sim_t=5760s) hard_guard=0
[INFO] [1781267991.632347191]: [batch] 64800/72000 ticks (sim_t=6480s) hard_guard=0
[INFO] [1781267991.677039837]: [batch] 72000/72000 ticks (sim_t=7200s) hard_guard=0
[WARN] [1781267991.677095207]: [END] ====== 结尾全队快照 ======
tick=72000
  V0 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=9->9 s=5.338/5.338 rem=0.000 spd=0.000 wait=0.0 gen=1
  V1 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=10->10 s=5.156/5.156 rem=0.000 spd=0.000 wait=0.0 gen=1
  V2 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=8->8 s=4.899/4.899 rem=0.000 spd=0.000 wait=0.0 gen=1
  V3 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=7->7 s=4.461/4.461 rem=0.000 spd=0.000 wait=0.0 gen=1
  V4 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=40->40 s=4.179/4.179 rem=0.000 spd=0.000 wait=0.0 gen=1
  V5 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=55->55 s=4.160/4.160 rem=0.000 spd=0.000 wait=0.0 gen=1
  V6 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=42->42 s=3.722/3.722 rem=0.000 spd=0.000 wait=0.0 gen=1
  V7 mode=2 act=0 reason=not_active blk=-1 brkr=0 task=1 slot=53->53 s=3.702/3.702 rem=0.000 spd=0.000 wait=0.0 gen=1
[WARN] [1781267991.677136354]: [batch] ==== 汇总: ticks=72000 sim_t=7200s | 碰撞(hard_guard)事件=0 首次@tick=0 涉及对=[] | 死锁检出拍=0 重规划脱困=0 | 最大wait=0.0s(V-1) ====
================================================================================REQUIRED process [multi_vehicle_patrol_node-2] has died!
process has finished cleanly
log file: /home/lsj/.ros/log/cd309fd0-665b-11f1-83c7-2149dec364f4/multi_vehicle_patrol_node-2*.log
Initiating shutdown!
================================================================================
[multi_vehicle_patrol_node-2] killing on exit
[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done