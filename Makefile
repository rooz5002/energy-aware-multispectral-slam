PY := env -u PYTHONPATH .venv/bin/python
SEQUENCES := office_loop_tune office_loop_01 corridor_tunnel_01 industrial_hall_01

.PHONY: paper-figures test worlds record calibrate calibrate-q calibrate-profiles calibrate-thermal-sparse tune orbslam3 integration-test run aggregate

test:
	$(PY) -m pytest -q tests

# robot model, textures, world SDFs and routes (deterministic)
worlds:
	python3 sim/models/build_robot_model.py
	cd sim && $(PY) -m worldgen.build_worlds

# record + export one sequence:  make record SEQ=office_loop_01
record:
	sim/record_sequence.sh $(SEQ)
	bash -c 'source env.sh && python3 harness/export_sequence.py data/sequences/$(SEQ)'
	$(PY) -m harness.check_keypoints data/sequences/$(SEQ)

calibrate:
	$(PY) -m harness.calibrate data/sequences/office_loop_tune

# ORB-SLAM3 at the pinned commit + patch + Python binding
orbslam3:
	orbslam3_patch/build.sh

integration-test:
	$(PY) -m pytest -q -m integration tests/test_integration.py

# one run (development):  make run SEQ=office_loop_tune PROFILE=fog_severe CONFIG=adaptive
run:
	$(PY) -m harness.run_one --sequence $(SEQ) --profile $(PROFILE) --config $(CONFIG) $(ARGS)

aggregate:
	$(PY) -m harness.aggregate

# calibration on the tuning sequence, in this order (after recording it)
calibrate-q:
	$(PY) -m harness.calibrate_q

calibrate-profiles:
	$(PY) -m harness.calibrate_profiles

calibrate-thermal-sparse:
	$(PY) -m harness.calibrate_thermal_sparse

# M7: tuning on the tuning sequence (needs the D13 measurement conditions), writes config/frozen.yaml
tune:
	$(PY) -m tuning.tune

# paper tables and figures from the per-run files in results/v5 (no run directories or recorded data needed):
# Table II = table_parameters, Table III = table_main, Figs. 3-5 = fig_a, fig_b, fig_c
paper-figures:
	$(PY) -m harness.paper_outputs --from-tables results/v5
